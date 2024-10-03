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

    def info_callback(self, msg):
        self.detection.conf = msg.conf
        self.detection.label = msg.label
        self.detection.position = msg.position
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
        self.get_logger().info(f"AAAAAAAAAAAAAAAA{self.detection.label}, {self.detection.conf} {self.rotation}")
        if self.detection.label == 2 and self.detection.conf > 65:
#            self.current_goal = self.waypoints[self.wp]
            self.get_logger().info("Person found!")
            time.sleep(5)
            self.finish(True, 1.0, "Person found " + str(self.location))
        else:
             self.get_logger().info("Looking for person")
             self.send_feedback(self.progress_, 'Search running')
             # Implement rotation for the robot
             if(self.rotation > 360):
                 self.rotation = 0
                 self.get_logger().info("No person found!")
                 time.sleep(5)
                 self.finish(True, 1.0, "No found " + str(self.location))
             else:
                 msg = Rotate()
                 msg.rotation.data = self.rotation
                 self.rotation = self.rotation + 18
                 self.publisher_rotation.publish(msg)
    '''
    def do_work(self):
        if self.progress_ < 1.0:
            self.progress_ += 0.05
            self.send_feedback(self.progress_, 'Search running')
            #self.get_logger().info("AAAAAAAAAAAAAAAAAAAAAAAAAAAA {}".format(self.detection.label))
        else:
            #self.finish(True, 1.0, 'Search completed');
            self.progress_ = 0.0
            found_person = random() > 0.3
            self.get_logger().info('Busqueda completada. Persona encontrada: {}'.format(found_person))
            if (found_person):
              self.finish(True, 1.0, 'Person found ' + str(self.location))
            else:
              self.finish(True, 1.0, 'No found ' + str(self.location))

        self.get_logger().info('searching ... {}'.format(self.progress_))
    '''
def main(args=None):
    rclpy.init(args=args)

    node = SearchAction()
    node.set_parameters([Parameter(name='action_name', value='search')])

    node.trigger_configure()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

