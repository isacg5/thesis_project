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
from plansys2_msgs.msg import ActionExecution
import time

class MoveAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('move', 0.5)
        self.progress_ = 0.0
        self.location = None

        self.subscription = self.create_subscription(
            ActionExecution,
            '/actions_hub', self.listener_callback, 10)


    def listener_callback(self, msg):
        parameters = msg.arguments
        action_name = msg.action
#        self.get_logger().info(f'Action received with parameters {parameters}')

        # Here, perform any action-specific handling based on action_name and parameters
        if action_name == 'move':
            if len(parameters) == 3:  # Expected 'move <robot> <from> <to>'
                self.location = parameters[2]
#                self.get_logger().info(f'Robot in {self.location}')

    def do_work(self):
        if self.progress_ < 0.5:
            self.progress_ += 0.05
            self.send_feedback(self.progress_, 'Move running')
        else:
            self.get_logger().info('moved ... {}'.format(self.location))
            self.send_feedback(self.progress_, 'Moved ' + self.location)
            time.sleep(1)
            self.finish(True, 1.0, 'Finished moved');
            self.progress_ = 0.0

        self.get_logger().info('moving ... {}, {}'.format(self.progress_, self.location))

def main(args=None):
    rclpy.init(args=args)

    node = MoveAction()
    node.set_parameters([Parameter(name='action_name', value='move')])

    node.trigger_configure()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
