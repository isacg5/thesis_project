#!/usr/bin/env python3
import rclpy
from rclpy.parameter import Parameter
from custom_msg_srv.srv import GetBatteryState
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from random import random

class BatteryStateClient:

    def __init__(self):
        self.node = rclpy.create_node('battery_state_client')
        self.client = self.node.create_client(GetBatteryState, 'get_battery_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting again...')
        self.node.get_logger().info('Service available!')

    def get_battery_state(self):
        request = GetBatteryState.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            return future.result().battery_state
        else:
            self.node.get_logger().error('Failed to get battery state')
            return None

    def __del__(self):
        self.node.destroy_node()
        rclpy.shutdown()


class CheckAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('check', 0.5)
        self.progress_ = 0.0
        self.battery_client = BatteryStateClient()

    def do_work(self):
        self.get_logger().info('Executing do_work function...')

        # Request battery state
        battery_state = self.battery_client.get_battery_state()
        if battery_state is not None:
            self.get_logger().info(f'Battery state: {battery_state}')
        else:
            self.get_logger().error('Failed to get battery state')
            return

        # Example processing based on battery state
        if battery_state > 20:
            self.finish(True, battery_state, 'Enough battery')
        else:
            self.finish(True, battery_state, 'Low battery')

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
