
#!/usr/bin/python3

# Copyright 2023 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import bosdyn.client
import bosdyn.client.util
from bosdyn.api import basic_command_pb2
from bosdyn.api import geometry_pb2 as geo
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME,
                                         get_se2_a_tform_b)
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_for_trajectory_cmd, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient
import numpy as np

import rclpy
from rclpy.parameter import Parameter, ParameterType
from collections import Counter
import numpy as np
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from plansys2_msgs.msg import ActionExecution
from zed.msg import Evaluation
from zed.msg import StructureArray
from zed.msg import Structure
import time
import math
from custom_msg_srv.msg import Rotate

THRESHOLD_HIP_SHOULDER_STAND = 210
THRESHOLD_KNEE_HIP_STAND = 190
THRESHOLD_ANKLE_KNEE_STAND = 230

THRESHOLD_HIP_SHOULDER_SIT = 200
THRESHOLD_KNEE_HIP_SIT = -100
THRESHOLD_ANKLE_KNEE_SIT = 50

ANGLE_SIT_THRESHOLD = 140


class EvaluateAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('evaluate', 0.5)
        self.sub_info = self.create_subscription(Evaluation, '/evaluation', self.evaluation_clbk, 10)

        self.positions = []
        self.info = StructureArray()


        self.start = True
        self.rotation = False
        self.request = False


    def evaluation_clbk(self, msg):
        pos = []
        for i in msg.poses:
            pos.append([i.x, i.y])

        result = []

        result.append(self.general_ev(pos[2], pos[8], pos[9], pos[10]))
        result.append(self.general_ev(pos[5], pos[11], pos[12], pos[13]))

        angle_l, angle_r = self.angle_hips(pos)

        if(angle_l != None):
            a = self.evaluate_angle(angle_l)
            if(a == "SIT"):
               result.append(a)
        if(angle_r != None):
            b = self.evaluate_angle(angle_r)
            if(b == "SIT"):
               result.append(b)

        dist_l, dist_r = self.nose_ankle(pos)
        result.append(dist_l)
        result.append(dist_r)

        filtered = [st for st in result if st is not None]

        if not filtered:
            common = "Unknown"
        else:
            cont = Counter(filtered)
            common = cont.most_common(1)[0][0]
        if(common != None):
            common = common.lower()
        self.final_result = common

        print("RESULT : ", result , " ", common, "\n")
        #self.finished = True

    def general_ev(self, shoulder, hip, knee, ankle):

        if(shoulder != [-1.0, -1.0] and hip != [-1.0, -1.0] and ankle != [-1.0, -1.0]):
            hip_shoulder_y = hip[1] - shoulder[1]
            knee_hip_y = knee[1] - hip[1]
            ankle_knee_y = ankle[1] - knee[1]

            hip_shoulder_x = hip[0] - shoulder[0]
            knee_hip_x = knee[0] - hip[0]
            ankle_knee_x = ankle[0] -knee[0]

            if(abs(hip_shoulder_x) > abs(hip_shoulder_y)):
                return "LAY"

            if(hip_shoulder_y > THRESHOLD_HIP_SHOULDER_STAND and knee_hip_y > THRESHOLD_KNEE_HIP_STAND and ankle_knee_y > THRESHOLD_ANKLE_KNEE_STAND): 
                return "STAND"

            elif(hip_shoulder_y > THRESHOLD_HIP_SHOULDER_SIT and knee_hip_y > THRESHOLD_KNEE_HIP_SIT and knee_hip_y < THRESHOLD_KNEE_HIP_STAND):
                return "SIT"

            else:
                return None
        else:
            return None


    def angle_hips(self, pos):
        a1 = None
        a2 = None
        if(pos[5] != [-1.0, -1.0] and pos[11] != [-1.0, -1.0] and pos[12] != [-1.0, -1.0]):
            a1 = self.get_angle(pos[5], pos[11], pos[12])

        if(pos[2] != [-1.0, -1.0] and pos[8] != [-1.0, -1.0] and pos[9] != [-1.0, -1.0]):
            a2 = self.get_angle(pos[2], pos[8], pos[9])

        return a1, a2


    def get_angle(self, A, B, C):
        BA = np.array([A[0] - B[0], A[1] - B[1]])
        BC = np.array([C[0] - B[0], C[1] - B[1]])

        # Dot product
        dot_product = np.dot(BA, BC)

        # Magnitudes
        magnitude_BA = np.linalg.norm(BA)
        magnitude_BC = np.linalg.norm(BC)

        # Cosine of the angle
        cos_angle = dot_product / (magnitude_BA * magnitude_BC)

        # Angle in radians
        angle_radians = np.arccos(np.clip(cos_angle, -1.0, 1.0))  # Clip to handle floating point errors

        # Angle in degrees
        angle_degrees = np.degrees(angle_radians)


    def evaluate_angle(self, angle):
        if(angle > ANGLE_SIT_THRESHOLD):
            return "OTHER"
        else:
            return "SIT"

    def evaluate_nose_ankle_distance(self, dx, dy):
        dif = abs(dx - dy)
        dx = abs(dx)
        dy = abs(dy)

        if(dx < dy and dy > 700 and dif > 200):
            return "STAND"
        elif(dx < dy and dy > 300 and dy < 700 and dif > 200):
            return "SIT"
        elif(dx > dy and dif > 300):
            return "LAY"

    def nose_ankle(self, pos):
        a = None
        b = None
        # If the distance in x is smaller than the distance in y, it means the person is vertically located
        if(pos[0] != [-1.0, -1.0] and pos[13] != [-1.0, -1.0]):
            d1x, d1y = pos[0][0] - pos[13][0], pos[0][1] - pos[13][1]
            a = self.evaluate_nose_ankle_distance(d1x, d1y)

        if(pos[0] != [-1.0, -1.0] and pos[10] != [-1.0, -1.0]):
            d2x, d2y = pos[0][0] - pos[10][0], pos[0][1] - pos[10][1]
            b = self.evaluate_nose_ankle_distance(d2x, d2y)

        return a, b



    def do_work(self):
        print("Evaluating")



def main(args=None):
    rclpy.init(args=args)

    node = EvaluateAction()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
