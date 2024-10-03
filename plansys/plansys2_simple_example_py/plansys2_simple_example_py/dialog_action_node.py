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
import re
from custom_msg_srv.msg import Rotate
import random


class DialogAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('dialog', 0.5)
        self.progress_ = 0.0
        self.location = None
        self.publisher_rotation = self.create_publisher(Rotate, '/rotation', 10)

    def ask_question(self, question):
        print(question)
        response = 'monday'
        #response = input("Your answer: ").strip().lower()
        return response

    def is_valid_day(self, day):
        valid_days = ["monday", "tuesday", "wednesday", "thursday", "friday", "saturday", "sunday"]
        return day in valid_days

    def is_valid_name(self, name):
        return bool(name) and name.isalpha() and len(name) > 1

    def check_consciousness(self):
        print("Starting consciousness test...")

        # Question 1: Name
        name = self.ask_question("What is your name?")
        if not name or not self.is_valid_name(name):
            print("Invalid or no response received for the name.")
            return False

        # Question 2: Day of the week
        day_of_week = self.ask_question("What day of the week is it today?")
        if not day_of_week or not self.is_valid_day(day_of_week):
            print("Invalid or no response received for the day of the week.")
            return False

        # Question 3: Location
        location = self.ask_question("Do you know where you are?")
        if not location:
            print("No response received for the location.")
            return False
        location = "yes"
        if location == "yes" :
            location = True
        else:
            location = False

        # Determine consciousness state
        if self.is_valid_name(name) and self.is_valid_day(day_of_week) and location:
            print("Thank you for your responses. Processing...")
            print("The person seems to be conscious.")
            return 'conscious'

        elif self.is_valid_name(name) and self.is_valid_day(day_of_week):
            print("The person seems to be confused. They answered the name and the day of the week correctly but don't know where they are.")
            return 'confused'

        elif self.is_valid_name(name):
            print("The person seems to be confused. They only know their name.")
            return 'confused'

        else:
            print("No consciousness detected.")
            return'unconscious'

    def do_work(self):
        if self.progress_ < 0.20:
            self.progress_ += 0.05
            self.send_feedback(self.progress_, 'Dialog running')
        else:
            result = self.check_consciousness()

            a = ['conscious', 'unconscious', 'confused']
            result = random.choice(a)

            if (result == 'conscious'):
               self.send_feedback(self.progress_, 'Diastate conscious')
            elif (result == 'confused'):
               self.send_feedback(self.progress_, 'Diastate confused')
            else:
               self.send_feedback(self.progress_, 'Diastate unconscious')

            time.sleep(5)
            self.get_logger().info('dialog response ... {}'.format(result))

            msg = Rotate()
            msg.rotation.data = 0.0
            #self.publisher_rotation.publish(msg)

            self.finish(True, 1.0, 'end dialog');

            self.get_logger().info('dialog running ... {}'.format(self.progress_))


def main(args=None):
    rclpy.init(args=args)

    node = DialogAction()
    node.set_parameters([Parameter(name='action_name', value='dialog')])

    node.trigger_configure()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
