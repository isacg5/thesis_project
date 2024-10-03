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
import csv

from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from random import random
from plansys2_msgs.msg import ActionExecution
from rclpy.node import Node
from plansys2_msgs.srv import AffectNode, AffectParam, GetProblem, GetDomain
from plansys2_msgs.msg import Node as NodeMsg
from plansys2_msgs.msg import Param
from threading import Event
import time
from std_msgs.msg import Bool
from zed.msg import StructureArray
from zed.msg import Structure

class ReportAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('report', 0.5)
        self.progress_ = 0.0
        self.finished = False
        self.sub_info = self.create_subscription(StructureArray, '/reported_info', self.info_callback, 10)
        self.info = StructureArray()
        self.file_name = "/home/linuxbrew/ros2_ws/src/plansys/plansys2_simple_example_py/plansys2_simple_example_py/StatusReport.txt"

    def info_callback(self, msg):
        #for i in range(len(msg.people_information)):
        #    self.info = StructureArray()
        self.info = msg
        if(len(self.info.people_information) > 0):
            self.get_logger().info("RECCCCC")
            self.finished = True

    def do_work(self):
        self.send_feedback(0.0, 'Report running')
        if(self.progress_ <= 0.1):
            self.progress_ += 0.05
            self.get_logger().info('reporting state ... {}'.format(self.progress_))

        else:
            self.progress = 0.0
            self.finished = False
            with open(self.file_name, "w") as file:
                writer = csv.writer(file)
                if file.tell() == 0:
                    writer.writerow(["person", "location", "position", "consciousness"])
                if(len(self.info.people_information) > 0):
                    for i in range(len(self.info.people_information)):
                        writer.writerow([self.info.people_information[i].person, self.info.people_information[i].location, self.info.people_information[i].state, self.info.people_information[i].consciousness])

            self.send_feedback(self.progress_, 'State reported')
            time.sleep(1)
            self.finish(True, 1.0, 'Finished reported')
        '''
        if self.progress_ < 1.0:
            self.progress_ += 0.05
            a = len(self.info.people_information)
            self.get_logger().info('reporting state ... {}'.format(a))
            self.send_feedback(self.progress_, 'Report running')
        else:
            #self.finish(True, 1.0, 'Search completed');
            self.progress_ = 0.0
            self.get_logger().info("FFFFFINISHED")
            self.finish(True, 1.0, 'State reported')
        '''
        self.get_logger().info('reporting state ... {}'.format(self.progress_))


def main(args=None):
    rclpy.init(args=args)

    node = ReportAction()
    node.set_parameters([Parameter(name='action_name', value='report')])

    node.trigger_configure()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
