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

from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from random import random
from plansys2_msgs.msg import ActionExecution
from rclpy.node import Node
from plansys2_msgs.srv import AffectNode, AffectParam, GetProblem, GetDomain
from plansys2_msgs.msg import Node as NodeMsg
from plansys2_msgs.msg import Param
from threading import Event
import time

class SearchAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('search', 0.5)
        self.progress_ = 0.0
        self.location = None
        self.c = 1

        self.subscription = self.create_subscription(
            ActionExecution,
            '/actions_hub', self.listener_callback, 10)

        self.people = []
        self.d = random() > 0.7
        #self.d = True

    def listener_callback(self, msg):
        parameters = msg.arguments
        action_name = msg.action
#        self.get_logger().info(f'Action received with parameters {parameters}')

        # Here, perform any action-specific handling based on action_name and parameters
        if action_name == 'search':
            if len(parameters) == 2:  # Expected 'search <robot> <location>'
                self.location = parameters[1]
#                self.get_logger().info(f'Robot in {self.location}')


    def do_work(self):
        if self.progress_ < 1.0 and self.d:
            self.progress_ += 0.05
            if(self.c == 10):
                self.get_logger().info("AAAAAA")
                f = random() > 0.5
                if(f):
                    self.send_feedback(self.progress_, 'Person ' + str(self.location) + ' p1 [4,2,5]')

            if(self.c == 5):
                self.get_logger().info("abababababa")
                f = random() > 0.5
                if(f):

                    self.send_feedback(self.progress_, 'Person ' + str(self.location) + ' p0 [4,2,5]')

            if(self.c == 20):
                self.c = 0
                self.get_logger().info("BBBBB")
                f = random() > 0.5
                #f = True
                if(f):
                    self.send_feedback(self.progress_, 'Person ' + str(self.location) + ' p3 [3,8,4]')

            if(self.c == 15):
                self.get_logger().info("abababababa")
                f = random() > 0.5
                if(f):
                    self.send_feedback(self.progress_, 'Person ' + str(self.location) + ' p2 [4,2,5]')

            """
            if(self.c == 25):
                self.get_logger().info("abababababa")
                self.send_feedback(self.progress_, 'Person ' + str(self.location) + ' p4 [4,2,5]')

            if(self.c == 30):
                self.get_logger().info("abababababa")
                self.send_feedback(self.progress_, 'Person ' + str(self.location) + ' p5 [4,2,5]')

            if(self.c == 35):
                self.get_logger().info("abababababa")
                self.send_feedback(self.progress_, 'Person ' + str(self.location) + ' p6 [4,2,5]')

            if(self.c == 40):
                self.get_logger().info("abababababa")
                self.send_feedback(self.progress_, 'Person ' + str(self.location) + ' p7 [4,2,5]')


            if(self.c == 45):
                self.get_logger().info("abababababa")
                self.send_feedback(self.progress_, 'Person ' + str(self.location) + ' p8 [4,2,5]')

            if(self.c == 50):
                self.get_logger().info("abababababa")
                self.send_feedback(self.progress_, 'Person ' + str(self.location) + ' p9 [4,2,5]')

            if(self.c == 55):
                self.c = 0
                self.get_logger().info("abababababa")
                self.send_feedback(self.progress_, 'Person ' + str(self.location) + ' p10 [4,2,5]')
            """

            self.c = self.c + 1

        else:
            self.d = random() > 0.2
            #self.finish(True, 1.0, 'Search completed');
            self.progress_ = 0.0
            found_person = random() > 0.2
            self.send_feedback(self.progress_, 'No person found here')
            time.sleep(5)
            self.get_logger().info('Busqueda completada. Persona encontrada: {}'.format(found_person))
            self.finish(True, 1.0, "Busqueda terminada a b")
            #if (found_person):
            #  self.finish(True, 1.0, 'Person found ' + str(self.location))
            #else:
            #  self.finish(True, 1.0, 'No found ' + str(self.location))

        self.get_logger().info('searching ... {}'.format(self.progress_))


def main(args=None):
    rclpy.init(args=args)

    node = SearchAction()
    node.set_parameters([Parameter(name='action_name', value='search')])

    node.trigger_configure()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
