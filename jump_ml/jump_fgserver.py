#### imports ####
import sys
import os
import asyncio
from base64 import standard_b64encode
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer
import time
import threading
import numpy as np

#### import gz trasport ####
from gz.transport13 import Node


####import custom gz msg #####
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.transport13 import Node

current_path = os.path.dirname(os.path.abspath(__file__))

sys.path.append(current_path + "/../jump_msgs/build/jump_msgs-msgs_genmsg/python")
from jump.msgs.matrix_pb2 import VectorMsg

#### create the suscom schema ####

with open(
    os.path.join(
        current_path + "/../jump_msgs/proto/jump/msgs",
        "MatrixMsg.bin",
    ),
    "rb",
) as schema_bin:
    schema_base64 = standard_b64encode(schema_bin.read()).decode("ascii")


class FoxGloveServer(object):
    def __init__(self):
        # print("ENTER")
        self.new_msg_flag = False
        self.msg = VectorMsg()
        # self.msgT = VectorMsg()
        # self.msg.data.append(0)
        # self.msg.data.append(0)
        self.msg_lock = threading.Lock()
        threading.Thread(target=self.entryPoint).start()

        self.node = Node()
        robot_states_topic = "/JumpRobot/ModelStates"

        if self.node.subscribe(VectorMsg, robot_states_topic, self.VectorMsg_cb):
            print(
                "Subscribing to type {} on topic [{}]".format(
                    VectorMsg, robot_states_topic
                )
            )
        else:
            print("Error subscribing to topic [{}]".format(robot_states_topic))
            return

        self.zeros = np.array((2, 1))

    def VectorMsg_cb(self, r_msg: VectorMsg):
        with self.msg_lock:
            # print(r_msg)
            self.new_msg_flag = True
            self.msg.CopyFrom(r_msg)

    def entryPoint(self):
        asyncio.run(self.main())

    async def main(self):
        async with FoxgloveServer("0.0.0.0", 8765, "example server") as server:
            chan_id = await server.add_channel(
                {
                    "topic": "example_msg",
                    "encoding": "protobuf",
                    "schemaName": "jump.msgs.VectorMsg",
                    "schema": schema_base64,
                }
            )

            time_to_sleep = 1
            while True:
                await asyncio.sleep(time_to_sleep)
                with self.msg_lock:
                    if self.new_msg_flag:
                        # print(self.msg.data[0])
                        await server.send_message(
                            chan_id,
                            time.time_ns(),
                            self.msg.SerializeToString(),
                        )
                        self.new_msg_flag = False
                        time_to_sleep = 0


def get_system():
    return FoxGloveServer()
