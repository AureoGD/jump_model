#### other imports ####
import sys
import os
import random
import time

#### gz imports ####
from gz.sim8 import Model, Joint
from gz.transport13 import Node

#### Import the jump model object ####
import jump_model

#### icecream for debug ####
from icecream import ic

#### import the pybind interface ####
current_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_path + "/../jump_rgc/build/python")
import jump_interface_py

#### import gz msgs ####
# os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.world_control_pb2 import WorldControl

#### import the custom msg ####
sys.path.append(current_path + "/../jump_msgs/build/jump_msgs-msgs_genmsg/python")
from jump.msgs.matrix_pb2 import VectorMsg


class JumpSystem(object):
    def __init__(self):
        # "import" the robo model, params and agent
        self.robot_model = jump_model.JumpModel()
        # create a empty list
        self.joints = []

        # create the node object
        self.node = Node()

        # create the messages of the service "/world/default/control"
        self.request_reset = WorldControl()
        self.response_reset = Boolean()
        self.timeout_reset = 1000

        # create the mensagens of the service "/RobotInfos"
        self.request_info = Boolean()
        self.response_info = VectorMsg()
        self.timeout_info = 2000

        # variable to control the learning call
        self.n_int = -1

        # Create the service using the pybind11
        self.service_node = jump_interface_py.JumpInterface(
            "/ChoseAction", self.cb_service
        )

    def configure(self, entity, sdf, ecm, event_mgr):
        # check if the curent model is valid
        self.model = Model(entity)
        if not self.model.valid(ecm):
            raise RuntimeError(f"Model {entity} is invalid")

        # fill the joint list
        for _joint in self.robot_model.joint_name:
            ic(_joint, " - ", self.model.joint_by_name(ecm, _joint))
            ic(self.joints.append(Joint(self.model.joint_by_name(ecm, _joint))))

    def cb_service(self):
        # ic("req")
        self.service_node.action = self.robot_model.action(self.service_node.statesV)
        self.n_int = 0

    def post_update(self, info, ecm):
        # if paused, do nothing
        if info.paused:
            return

        # if the choose action was called once
        if self.n_int != -1:
            self.n_int += 1
            # check if is necessary to call the learning algorith
            if self.n_int == self.robot_model.step_size:
                # increment the current step
                self.robot_model.steps += 1
                # request the infos from the robot
                self.request_info.data = True
                # /RobotLearningStates
                result, self.response_info = self.node.request(
                    "/RobotLearningStates",
                    self.request_info,
                    Boolean,
                    VectorMsg,
                    self.timeout_info,
                )
                # ic(result)
                if result:
                    # decode the message
                    self.robot_model.NewData(self.response_info)
                    # call fo the learning method
                    done = self.robot_model.learning()
                else:
                    # error
                    ic("ERROR")
                    done = 1

                # if is done, reset the simulation
                if done:
                    self.n_int = -1
                    self.robot_model.steps = 0
                    self.ep_reset(ecm)

    def ep_reset(self, ecm):
        # call for a service to pause the sim
        self.request_reset.pause = True
        result, self.response_reset = self.node.request(
            "/world/default/control",
            self.request_reset,
            WorldControl,
            Boolean,
            self.timeout_reset,
        )
        # if paused
        if result and self.response_reset.data:
            # randomize the joints positions
            for index in range(len(self.joints)):
                # ic(index)
                self.joints[index].reset_position(
                    ecm,
                    [
                        random.uniform(
                            self.robot_model.joint_p_min[index],
                            self.robot_model.joint_p_max[index],
                        )
                    ],
                )
            max_ep = self.robot_model.UpdateUtils()
            time.sleep(0.05)
            if not max_ep:
                # call to unpause the simulation
                self.request_reset.pause = False
                result, self.response_reset = self.node.request(
                    "/world/default/control",
                    self.request_reset,
                    WorldControl,
                    Boolean,
                    self.timeout_reset,
                )
                if result and self.response_reset.data:
                    print(
                        f"##### EPISODE {self.robot_model.n_ep}/{self.robot_model.max_ep} FINISHED-> EP. REWARD = {self.robot_model.utils.curent_reward} / MEAM EP. REWARD = {self.robot_model.utils.curent_m_reward} #####"
                    )
                else:
                    print("##### ERROR IN RESET SERVICE CALL - UNPAUSE #####")
            else:
                print(
                    f"##### EPISODE {self.robot_model.n_ep} FINISHED-> EP. REWARD = {self.robot_model.utils.curent_reward} / MEAM EP. REWARD = {self.robot_model.utils.curent_m_reward} #####"
                )
                print("##### TRAINNING FINISHED #####")
        else:
            print("##### ERROR IN RESET SERVICE CALL - PAUSE #####")
