#!/usr/bin/python3
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


class EvaluateAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('evaluation', 0.5)
        self.progress_ = 0.0
        self.location = None

    def do_work(self):
        if self.progress_ < 0.20:
            self.progress_ += 0.05
            self.send_feedback(self.progress_, 'Evaluate running')
        else:
            a = ['lay', 'sit', 'stand']
            result = random.choice(a)

            if (result == 'lay'):
               self.send_feedback(self.progress_, 'Person state lay')
            elif (result == 'sit'):
               self.send_feedback(self.progress_, 'Person state sit')
            else:
               self.send_feedback(self.progress_, 'Person state stand')

            time.sleep(5)
            self.get_logger().info('Evaluate response ... {}'.format(result))

            self.finish(True, 1.0, 'end evaluate sit');

            self.get_logger().info('evaluate running ... {}'.format(self.progress_))


def main(args=None):
    rclpy.init(args=args)

    node = EvaluateAction()
    node.set_parameters([Parameter(name='action_name', value='evaluate')])

    node.trigger_configure()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
