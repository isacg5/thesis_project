#!/usr/bin/env python3
import rclpy
from rclpy.parameter import Parameter
from custom_msg_srv.srv import GetBatteryState
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from plansys2_msgs.msg import ActionExecution
from random import random
import time
import math
from std_msgs.msg import Float32

class CheckAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('check', 0.5)
        self.progress_ = 0.0
        self.subscription = self.create_subscription(
            ActionExecution,
            '/actions_hub', self.listener_callback, 10)
        self.sub_info = self.create_subscription(Float32, '/battery_info', self.battery_callback, 10)
        self.loc = {"a":(0.0, 0.0),"b":(-1.0, 0.5),"c":(-0.2, 0.5),"battery_point":(0.0, 0.0), "exit":(0.0, 0.0)}
        self.battery_state = None


    def battery_callback(self, msg):
        self.battery_state = msg.data

    def listener_callback(self, msg):
        parameters = msg.arguments
        action_name = msg.action
#        self.get_logger().info(f'Action received with parameters {parameters}')

        # Here, perform any action-specific handling based on action_name and param>
        if action_name == 'check':
            if len(parameters) == 3:  # Expected 'search <robot> <location>'
                self.destination = parameters[2]
                self.origin = parameters[1]
#                self.get_logger().info(f'Robot in {self.location}')


    def get_distance(self, bat = False):
        origin_point = self.loc[self.origin]
        dest_point = self.loc[self.destination]
        self.get_logger().info(f'from {self.origin} to {self.destination}')

        if(bat == True):
            dist = math.sqrt((dest_point[0] - 0)**2 + (dest_point[1] - 0)**2)
            return dist
        else:
            dist = math.sqrt((dest_point[0] - origin_point[0])**2 + (dest_point[1] - origin_point[1])**2)
            return dist

    def do_work(self):
        self.get_logger().info('Executing do_work function...')
        if self.progress_ < 0.25:
            self.progress_ += 0.05
        else:
            if self.battery_state is not None:
                self.get_logger().info(f'Battery state: {self.battery_state}')
            else:
                self.get_logger().error('Failed to get battery state')
                return

            # Example processing based on battery state
            #time.sleep(5)
            self.progress_ = 0.0
            dist_bat = self.get_distance(bat = True)
            dist = self.get_distance() # 10m for 1%

            self.get_logger().info(f'Distance {dist_bat + dist}')

            if((self.battery_state * 10) <= (dist_bat + dist) or (self.battery_state < 15)):
                self.send_feedback(self.battery_state, 'Low battery')
                time.sleep(1)
                self.finish(True, self.battery_state, 'End battery')
            else:
                self.send_feedback(self.battery_state, 'Enough battery')
                time.sleep(1)
                self.finish(True, self.battery_state, 'End battery')

           #if battery_state > 15:
           #     self.finish(True, battery_state, 'Enough battery')
           # else:
           #     self.finish(True, battery_state, 'Low battery')

def main(args=None):
    rclpy.init(args=args)
    node = CheckAction()
    node.set_parameters([Parameter(name='action_name', value='check')])
    node.trigger_configure()
    #node.trigger_activate()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
