import os

# for keras the CUDA commands must come before importing the keras libraries
os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
os.environ["TF_FORCE_GPU_ALLOW_GROWTH"] = "true"

import numpy as np
from keras.optimizers import Adam
from keras.layers import Dense, Activation, Input
from keras.models import load_model
from keras import Sequential
from keras.layers import Dense, Dropout, Conv1D, MaxPooling2D, Activation, Flatten


class ReplayBuffer:
    def __init__(self, max_size, input_shape, n_actions, discrete=False):
        self.mem_size = max_size
        self.mem_cntr = 0
        self.discrete = discrete
        self.state_memory = np.zeros((self.mem_size, input_shape))
        self.new_state_memory = np.zeros((self.mem_size, input_shape))
        dtype = np.int8 if self.discrete else np.float32
        self.action_memory = np.zeros((self.mem_size, n_actions), dtype=dtype)
        self.reward_memory = np.zeros(self.mem_size)
        self.terminal_memory = np.zeros(self.mem_size, dtype=np.float32)

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


class DDQNAgent(object):
    def __init__(
        self,
        alpha,
        gamma,
        n_actions,
        epsilon,
        batch_size,
        input_dims,
        epsilon_dec=0.996,
        epsilon_end=0.01,
        mem_size=1000000,
        fname="ddqn_model.h5",
        replace_target=100,
    ):
        self.action_space = [i for i in range(n_actions)]
        self.n_actions = n_actions
        self.gamma = gamma
        self.alpha = alpha
        self.epsilon = epsilon
        self.epsilon_dec = epsilon_dec
        self.epsilon_min = epsilon_end
        self.batch_size = batch_size
        self.model_file = fname
        self.replace_target = replace_target
        self.input_dims = input_dims

        self.fc1_dims = 256
        self.fc2_dims = 256

        self.memory = ReplayBuffer(
            mem_size, self.input_dims, self.n_actions, discrete=True
        )

        self.q_eval = self.create_model()
        self.q_target = self.create_model()
        self.update_network_parameters()

    def create_model(self):
        model = Sequential()
        model.add(Input(shape=(self.input_dims,)))
        model.add(
            Dense(
                self.fc1_dims,
            )
        )
        model.add(Activation("relu"))
        model.add(Dense(self.fc2_dims))
        model.add(Activation("relu"))
        model.add(Dense(self.n_actions))

        model.compile(
            loss="mse", optimizer=Adam(learning_rate=self.alpha), metrics=["accuracy"]
        )

        return model

    def remember(self, state, action, reward, new_state, done):
        self.memory.store_transition(state, action, reward, new_state, done)

    def choose_action(self, state):
        state = state[np.newaxis, :]
        rand = np.random.random()
        if rand < self.epsilon:
            action = np.random.choice(self.action_space)
        else:
            actions = self.q_eval.predict(state, verbose=0)
            action = np.argmax(actions)

        return action

    def learn(self):
        if self.memory.mem_cntr > self.batch_size:
            state, action, reward, new_state, done = self.memory.sample_buffer(
                self.batch_size
            )

            action_values = np.array(self.action_space, dtype=np.int8)
            action_indices = np.dot(action, action_values)

            q_next = self.q_target.predict(
                new_state, verbose=0, use_multiprocessing=True
            )
            q_eval = self.q_eval.predict(new_state, verbose=0, use_multiprocessing=True)
            q_pred = self.q_eval.predict(state, verbose=0, use_multiprocessing=True)

            max_actions = np.argmax(q_eval, axis=1)

            q_target = q_pred

            batch_index = np.arange(self.batch_size, dtype=np.int32)

            q_target[batch_index, action_indices] = (
                reward
                + self.gamma * q_next[batch_index, max_actions.astype(int)] * done
            )

            _ = self.q_eval.fit(state, q_target, verbose=0, use_multiprocessing=True)

            self.epsilon = (
                self.epsilon * self.epsilon_dec
                if self.epsilon > self.epsilon_min
                else self.epsilon_min
            )

            if self.memory.mem_cntr % self.replace_target == 0:
                self.update_network_parameters()

    def update_network_parameters(self):
        self.q_target.set_weights(self.q_eval.get_weights())

    def save_model(self):
        self.q_eval.save(self.model_file)

    def load_model(self):
        self.q_eval = load_model(self.model_file)
        # if we are in evaluation mode we want to use the best weights for q_target
        if self.epsilon == 0.0:
            self.update_network_parameters()
