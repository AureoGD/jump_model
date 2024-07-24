#### other imports ####
from math import pi, cos, sqrt
import numpy as np
import threading
import time

#### icecream for debug ####
from icecream import ic

#### import util for plot####
import utils

#### import the agent ####
from ddqn_torch import Agent
# from ddqn_keras import DDQNAgent


class CreatVectorData(object):
    def __init__(self, d_lenght, d_startpos):
        # self.data = np.random.rand(d_lenght, 1)
        self.data = np.zeros((d_lenght, 1), dtype=np.double)
        self.startpos = d_startpos
        self.lenght = d_lenght

    def UpdateData(self, d_source):
        for i in range(self.lenght):
            self.data[i] = d_source.data[self.startpos + i]


class JumpModel(object):
    def __init__(self):
        # TODO import this infos from the SDF file
        self.joint_name = ["base", "hfe", "kfe"]
        self.joint_type = ["prismatic", "revolute", "revolute"]
        self.joint_p_max = [0.9, 0 * pi / 180, 120 * pi / 180]
        self.joint_p_min = [0.5, -55 * pi / 180, 60 * pi / 180]

        nn_states_input_dim = 13
        n_actions = 6
        alpha = 0.0005
        gamma = 0.99
        epsilon = 1.0
        batch_size = 64
        epsilon_decay = 5e-6
        epsilon_min = 0.1
        mem_size = 1024
        replace_target = 100

        self.nn_input_dim = nn_states_input_dim

        # Predict horizon
        N = 10

        # sample time of the simulation
        sim_ts = 0.001
        # duration of one episode (seconds)
        episode_duration = 5

        self.dt = N * sim_ts

        # number of the current episode
        self.n_ep = 0
        # maximum episodes
        self.max_ep = 10000
        # number of episodes to save the model
        self.save_model_every = 10
        # number of the current step
        self.steps = 0
        # how many "post_update" is a step
        self.step_size = 10
        # how many steps is an episode
        self.max_steps = episode_duration / (sim_ts * self.step_size)

        # vector of the last states
        self.last_states = np.zeros((nn_states_input_dim, 1))
        self.teste_v = np.zeros((nn_states_input_dim + 1, 1))
        # vector of the actual state
        self.actual_state = np.zeros((nn_states_input_dim, 1))

        # vector of the last N actions
        self.actions = -np.ones((N, 1))

        # vector os the states
        self.base_pos = CreatVectorData(2, 0)  # 0  1
        self.base_vel = CreatVectorData(2, 2)  # 2  3
        self.com_pos = CreatVectorData(2, 4)  # 4  5
        self.com_vel = CreatVectorData(2, 6)  # 6  7
        self.foot_pos = CreatVectorData(2, 8)  # 8  9
        self.foot_vel = CreatVectorData(2, 10)  # 10 11
        self.joint_pos = CreatVectorData(2, 12)  # 12 13
        self.joint_vel = CreatVectorData(2, 14)  # 14 15
        self.joint_tau = CreatVectorData(2, 16)  # 16 17
        self.joint_refh = CreatVectorData(2, 18)  # 18 19
        self.joint_refl = CreatVectorData(2, 20)  # 20 21
        self.cons_viol = CreatVectorData(1, 22)  # 22
        self.foot_con = CreatVectorData(1, 23)  # 23
        self.npo = CreatVectorData(1, 24)  # 24

        self.states_list = [
            self.base_pos,
            self.base_vel,
            self.com_pos,
            self.com_vel,
            self.foot_pos,
            self.foot_vel,
            self.joint_pos,
            self.joint_vel,
            self.joint_tau,
            self.joint_refh,
            self.joint_refl,
            self.cons_viol,
            self.foot_con,
            self.npo,
        ]

        self.base_vel_a = np.array(self.base_vel.data.shape)
        self.foot_vel_a = np.array(self.foot_vel.data.shape)

        self.utils = utils.Utils()
        self.episode_reward = 0
        self.episode_base_distance = 0
        self.episode_foot_distance = 0

        # finaly, create yhe agent
        self.agent = Agent(
            gamma=gamma,
            epsilon=epsilon,
            lr=alpha,
            n_actions=n_actions,
            input_dims=[nn_states_input_dim],
            mem_size=mem_size,
            batch_size=batch_size,
            eps_min=epsilon_min,
            eps_dec=epsilon_decay,
            replace=replace_target,
            chkpt_dir=self.utils.save_path,
        )

        self.weight_joint_e = 0.005
        self.weight_base_dist = 2.35
        self.weight_foot_dist = 2.25
        self.weight_jump = 2.5
        self.min_reward = -50

        self.time_jump = 0
        self.delta_jump = 0

        self.first_landing = 0

        qd_max = 100
        self.alfa = np.diagflat(
            [
                1,
                1,
                0.5 / 5,
                0.5 / 5,
                1,
                1,
                0.5 / 5,
                0.5 / 5,
                0.5 / 1.57,
                0.5 / 2.18,
                0.5 / qd_max,
                0.5 / qd_max,
                1 / N,
            ]
        )

        self.beta = np.array(
            [
                [0],
                [0],
                [0.5],
                [0.5],
                [0],
                [0],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0],
            ]
        )
        print(self.alfa.shape)
        print(self.beta.shape)

        self.transition_val = 0

    def action(self, states):
        # this method returns the choosed action
        self.actions[:] = np.roll(self.actions, 1)
        # self.CheckTransition()

        normalized_states = self.normalize_states(
            np.append(states.reshape(12, 1), [[self.CheckTransition()]], axis=0)
        ).reshape(
            self.nn_input_dim,
        )

        self.actions[0] = self.agent.choose_action(normalized_states)
        # save the current states for learning in the future
        self.last_states = normalized_states
        # return the choosed action
        return self.actions[0]

    def learning(self):
        # evaluate the reward of the last choosed action
        reward = self.eval_reward()
        # print(reward)
        if self.episode_reward + reward <= self.min_reward:
            self.episode_reward = reward
        else:
            self.episode_reward += reward

        # check if is done
        done = self.done(reward)

        # update the agent memory
        self.agent.remember(
            self.last_states.reshape(
                self.nn_input_dim,
            ),
            self.actions[0],
            reward,
            self.actual_state.reshape(
                self.nn_input_dim,
            ),
            done,
        )
        # agent learn
        self.agent.learn()
        # return done
        return done

    def eval_reward(self):
        reward = 0

        joint_e = self.joint_refh.data - self.joint_pos.data
        # ic(self.joint_refh.data, self.joint_pos.data, joint_e)
        for index in range(len(joint_e)):
            reward += self.weight_joint_e / sqrt(joint_e[index] ** 2)

        base_distance = (
            self.dt
            * (sqrt(self.base_vel.data[1] ** 2) + sqrt(self.base_vel_a[1] ** 2))
            / 2
        )

        # ic(self.base_vel_a.shape, self.base_pos.data.shape)
        self.base_vel_a[:] = self.base_vel.data[:, -1]

        self.episode_base_distance += base_distance
        reward += self.weight_base_dist * base_distance

        foot_distance = (
            self.dt
            * (
                sqrt(self.foot_vel.data[0] ** 2 + self.foot_vel.data[1] ** 2)
                + sqrt(self.foot_vel_a[0] ** 2 + self.foot_vel_a[1] ** 2)
            )
            / 2
        )
        self.foot_vel_a[:] = self.foot_vel.data[:, -1]

        self.episode_foot_distance += foot_distance
        reward += self.weight_foot_dist * foot_distance

        reward += -0.5 * (1 - self.cons_viol.data[0, 0])

        if self.foot_con and not self.first_landing:
            self.first_landing = 1

        if self.first_landing and not self.foot_con:
            self.time_jump = time.time()

        if self.first_landing and self.foot_con and self.time_jump != 0:
            self.delta_jump = time.time() - self.time_jump
            self.time_jump = 0
            reward += self.delta_jump * self.weight_jump + 5

        # ic(reward)

        # trans_val = self.CheckTransition()

        # ic(trans_val)

        reward += -0.005 * self.transition_val

        # terminal condition: if the knee is near of the ground
        if (
            self.base_pos.data[1, 0] - 0.285 * cos(self.joint_pos.data[0, 0]) - 0.125
            < 0.1
        ):
            reward = self.min_reward

        # print(self.base_pos.data[1, 0] + self.foot_pos.data[1, 0])
        if self.base_pos.data[1, 0] + self.foot_pos.data[1, 0] < 0:
            reward = self.min_reward

        return reward

    def done(self, reward):
        # ic(self.steps)
        # when finish the time of episode or the reward is too low
        if self.steps == self.max_steps or self.episode_reward <= self.min_reward:
            # print("true")
            return True

        # otherwise continue
        return False

    def NewData(self, msg):
        # detach the received data into small vectors
        for i in range(len(self.states_list)):
            self.states_list[i].UpdateData(msg)
        # Update the self.actual_state
        self.transition_val = self.CheckTransition()
        self.actual_state[0:2] = self.com_pos.data
        self.actual_state[2:4] = self.com_vel.data
        self.actual_state[4:6] = self.foot_pos.data
        self.actual_state[6:8] = self.foot_vel.data
        self.actual_state[8:10] = self.joint_pos.data
        self.actual_state[10:12] = self.joint_refl.data
        self.actual_state[12] = self.transition_val
        self.actual_state[:] = self.normalize_states(self.actual_state)

    def normalize_states(self, states):
        normalized_states = np.clip(
            np.matmul(self.alfa, states.reshape(self.nn_input_dim, 1)) + self.beta,
            0,
            1,
        )
        return normalized_states

    def CheckTransition(self):
        count = 0
        for i in range((len(self.actions) - 1), 0, -1):
            if self.actions[i] != self.actions[i - 1] and self.actions[i] != -1:
                count += 1
        if count == 1 or count == 0:
            return 0
        else:
            return count

    def UpdateUtils(self):
        self.n_ep += 1
        self.utils.updatePlot(
            self.agent.epsilon,
            self.episode_reward,
            self.episode_base_distance,
            self.episode_foot_distance,
        )

        self.episode_reward = 0
        self.episode_base_distance = 0
        self.episode_foot_distance = 0

        self.utils.saveLearningData()

        if not self.n_ep % self.save_model_every:
            self.agent.save_model()

        if self.n_ep == self.max_ep:
            return True

        return False
