#### imports ####
import os
import numpy as np
import time

import torch as T
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim


class ReplayBuffer:
    def __init__(self, max_size, input_shape, n_actions, discrete=False):
        self.mem_size = max_size
        self.mem_cntr = 0
        self.discrete = discrete
        self.state_memory = np.zeros((self.mem_size, *input_shape), dtype=np.float64)
        self.new_state_memory = np.zeros(
            (self.mem_size, *input_shape), dtype=np.float64
        )
        dtype = np.int8 if self.discrete else np.float64
        # self.action_memory = np.zeros((self.mem_size, n_actions), dtype=np.int32)
        self.action_memory = np.zeros(self.mem_size, dtype=np.int32)
        self.reward_memory = np.zeros(self.mem_size, dtype=np.float64)
        self.terminal_memory = np.zeros(self.mem_size, dtype=np.int32)

    def store_transition(self, state, action, reward, state_, done):
        index = self.mem_cntr % self.mem_size
        self.state_memory[index] = state
        self.new_state_memory[index] = state_
        # store one hot encoding of actions, if appropriate
        if self.discrete:
            actions = np.zeros(self.action_memory.shape[1])
            actions[action] = 1.0
            self.action_memory[index] = actions
        else:
            self.action_memory[index] = action
        self.reward_memory[index] = reward
        self.terminal_memory[index] = 1 - int(done)
        self.mem_cntr += 1

    def sample_buffer(self, batch_size):
        max_mem = min(self.mem_cntr, self.mem_size)
        batch = np.random.choice(max_mem, batch_size)

        states = self.state_memory[batch]
        actions = self.action_memory[batch]
        rewards = self.reward_memory[batch]
        states_ = self.new_state_memory[batch]
        terminal = self.terminal_memory[batch]

        return states, actions, rewards, states_, terminal


class DDQN_Network(nn.Module):
    def __init__(self, lr, input_dims, name, fc1_dims, fc2_dims, n_actions, chkpt_dir):
        super(DDQN_Network, self).__init__()

        self.checkpoint_dir = chkpt_dir
        self.checkpoint_file = os.path.join(self.checkpoint_dir, name)

        self.input_dims = input_dims
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.n_actions = n_actions
        self.fc1 = nn.Linear(*self.input_dims, self.fc1_dims, dtype=T.float64)
        self.fc2 = nn.Linear(self.fc1_dims, self.fc2_dims, dtype=T.float64)
        self.fc3 = nn.Linear(self.fc2_dims, self.n_actions, dtype=T.float64)

        self.optimizer = optim.Adam(self.parameters(), lr=lr)
        self.loss = nn.MSELoss()
        self.device = T.device("cuda:0" if T.cuda.is_available() else "cpu")
        # print(self.device)
        self.to(self.device)

    def forward(self, state):
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        actions = self.fc3(x)

        return actions

    def save_checkpoint(self):
        print("... saving checkpoint ...")
        T.save(self.state_dict(), self.checkpoint_file)

    def load_checkpoint(self):
        print("... loading checkpoint ...")
        self.load_state_dict(T.load(self.checkpoint_file))


class Agent:
    def __init__(
        self,
        gamma,
        epsilon,
        lr,
        n_actions,
        input_dims,
        mem_size,
        batch_size,
        eps_min=0.01,
        eps_dec=5e-4,
        replace=1000,
        chkpt_dir="tmp/dueling_ddqn",
    ):
        self.gamma = gamma
        self.epsilon = epsilon
        self.lr = lr
        self.n_actions = n_actions
        self.input_dims = input_dims
        self.batch_size = batch_size
        self.eps_min = eps_min
        self.eps_dec = eps_dec
        self.replace_target_cnt = replace
        self.chkpt_dir = chkpt_dir
        self.action_space = [i for i in range(self.n_actions)]
        self.learn_step_counter = 0

        self.memory = ReplayBuffer(mem_size, self.input_dims, self.n_actions)

        #
        self.q_eval = DDQN_Network(
            lr=self.lr,
            input_dims=self.input_dims,
            name="ddqn_q_eval",
            fc1_dims=512 * 2,
            fc2_dims=512 * 2,
            n_actions=self.n_actions,
            chkpt_dir=self.chkpt_dir,
        )

        self.q_next = DDQN_Network(
            self.lr,
            input_dims=self.input_dims,
            name="ddqn_q_next",
            fc1_dims=512 * 2,
            fc2_dims=512 * 2,
            n_actions=self.n_actions,
            chkpt_dir=self.chkpt_dir,
        )

    def choose_action(self, observation):
        if np.random.random() > self.epsilon:
            # state = T.tensor([observation], dtype=T.float64).to(self.q_eval.device)
            state = T.from_numpy(observation).to(self.q_eval.device)
            advantage = self.q_eval.forward(state)
            action = T.argmax(advantage).item()
        else:
            action = np.random.choice(self.action_space)

        return action

    def remember(self, state, action, reward, state_, done):
        self.memory.store_transition(state, action, reward, state_, done)

    def replace_target_network(self):
        if self.learn_step_counter % self.replace_target_cnt == 0:
            self.q_next.load_state_dict(self.q_eval.state_dict())

    def decrement_epsilon(self):
        self.epsilon = (
            self.epsilon - self.eps_dec if self.epsilon > self.eps_min else self.eps_min
        )

    def save_model(self):
        self.q_eval.save_checkpoint()
        # self.q_next.save_checkpoint()

    def load_models(self):
        self.q_eval.load_checkpoint()
        # self.q_next.load_checkpoint()

    def learn(self):
        if self.memory.mem_cntr < self.batch_size:
            return

        self.q_eval.optimizer.zero_grad()

        self.replace_target_network()

        state, action, reward, new_state, done = self.memory.sample_buffer(
            self.batch_size
        )

        states = T.from_numpy(state).to(self.q_eval.device)
        new_states = T.from_numpy(new_state).to(self.q_eval.device)
        actions = T.from_numpy(action).to(self.q_eval.device)
        reward = T.from_numpy(reward).to(self.q_eval.device)
        dones = T.from_numpy(done).to(self.q_eval.device)

        indices = np.arange(self.batch_size, dtype=np.int32)

        q_pred = self.q_eval.forward(states)[indices, actions]
        q_next = self.q_next.forward(new_states)
        q_eval = self.q_eval.forward(new_states)

        max_actions = T.argmax(q_eval, dim=1)

        q_next[dones] = 0.0

        q_target = reward + self.gamma * q_next[indices, max_actions]
        loss = self.q_eval.loss(q_target, q_pred).to(self.q_eval.device)
        loss.backward()
        self.q_eval.optimizer.step()
        self.learn_step_counter
        self.decrement_epsilon()
