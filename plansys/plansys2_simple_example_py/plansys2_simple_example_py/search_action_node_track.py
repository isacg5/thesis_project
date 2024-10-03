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
import time
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from plansys2_msgs.msg import ActionExecution
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from custom_msg_srv.msg import Rotate
import math
from zed.msg import Detection
import random

class SearchAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('search', 0.5)
        self.progress_ = 0.0
        self.location = None

        self.subscription = self.create_subscription(
            ActionExecution,
            '/actions_hub', self.listener_callback, 10)

        self.sub_info = self.create_subscription(Detection, '/position', self.info_callback, 10)
        self.detection = Detection()
        self.publisher_rotation = self.create_publisher(Rotate, '/rotation', 10)
        self.rotation = 0.0
        self.last = -1
        self.counter = 0
        #sdk = bosdyn.client.create_standard_sdk('VelodyneClient')
        #robot = sdk.create_robot('192.168.80.3')
        #robot.authenticate('user', 'wruzvkg4rce4')
        ##bosdyn.client.util.authenticate(robot)
        #robot.sync_with_directory()
        #self.robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    def info_callback(self, msg):
        self.detection.conf = msg.conf
        self.detection.position = msg.position
        self.detection.id = msg.id
        #self.get_logger().info(f'PERSON FOUND {self.detection.label}')


    def listener_callback(self, msg):
        parameters = msg.arguments
        action_name = msg.action
#        self.get_logger().info(f'Action received with parameters {parameters}')

        # Here, perform any action-specific handling based on action_name and param>
        if action_name == 'search':
            if len(parameters) == 2:  # Expected 'search <robot> <location>'
                self.location = parameters[1]
#                self.get_logger().info(f'Robot in {self.location}')


    def do_work(self):
        self.get_logger().info(f"AAAAAAAAAAAAAAAA {self.detection.conf} {self.rotation}")
        if self.detection.conf > 90 and self.last != self.detection.id:
            self.last = self.detection.id
#            self.current_goal = self.waypoints[self.wp]
            self.get_logger().info("Person found!")

           # x=60*0.001
           # y=115.274*0.001
           # z=-420.428*0.001

           # x_p = self.detection.position.x
           # y_p = self.detection.position.y
           # z_p = self.detection.position.z
           # cam_tform_body_mat= np.array([[0, -1, 0, x] ,
           #              [0, 0, -1, y],
           #             [1, 0, 0, z],
           #             [0, 0, 0, 1 ]])
           # cam_tform_body=bosdyn.client.math_helpers.SE3Pose.from_matrix(cam_tform_body_mat)
            #print(cam_tform_body)
           # body_tform_cam=cam_tform_body.inverse()

            #print(body_tform_cam)
           # transforms = self.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
           # odom_tform_body= bosdyn.client.frame_helpers.get_odom_tform_body(transforms)
           # odom_tform_cam= odom_tform_body * body_tform_cam
           # new_coords= odom_tform_cam.transform_point(x_p, y_p, z_p)
           # print("NEW COORDS: ", new_coords) 
           # angle = math.atan2(new_coords[0],new_coords[1])
           # print("ANGLE: ", angle, " ROTATION: ", self.rotation)
           # a = (angle*180)/3.1416

            if(self.rotation <= 36.0 and self.rotation >= 0.0):
                angle = self.rotation
            else:
                angle = self.rotation - 40
            #arr = '[' + str(self.detection.position.x) + ',' + str(self.detection.position.y) + ',' + str(self.detection.position.z) +']'
            arr = '[' + str(angle) + ',' + str(self.detection.position.y) + ',' + str(self.detection.position.z) +']'
            #arr = '[' + str(a) + ',' + str(self.detection.position.y) + ',' + str(self.detection.position.z) +']'

            feedback = 'Person ' + str(self.location) + ' p' +str(self.counter) + ' ' + arr
            self.send_feedback(self.progress_, feedback)
            self.get_logger().info(f"counter: {self.counter}, {arr}, {feedback}")
            self.counter += 1

            #time.sleep(5)
            #self.finish(True, 1.0, "Person found " + str(self.location)
        self.get_logger().info("Looking for person")
        #self.send_feedback(self.progress_, 'Search running')
        # Implement rotation for the robot
        if(self.rotation > 360):
            self.rotation = 0.0
            self.send_feedback(self.progress_, 'No person found here')
            time.sleep(5)
            self.get_logger().info('Busqueda completada. ')
            self.finish(True, 1.0, "Busqueda terminada a b")

            #self.get_logger().info("No person found here")
            #time.sleep(5)
            #self.finish(True, 1.0, "No found " + str(self.location))
        else:
            msg = Rotate()
            msg.rotation.data = self.rotation
            self.rotation = self.rotation + 18
            self.publisher_rotation.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = SearchAction()
    node.set_parameters([Parameter(name='action_name', value='search')])

    node.trigger_configure()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

