import rclpy
from rclpy.node import Node
from bosdyn.client import create_standard_sdk
from bosdyn.client.lease import LeaseClient
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandClient
from bosdyn.client.robot_id import RobotIdClient
from bosdyn.client.robot_state import RobotStateClient
import time
from std_msgs.msg import Float32
from custom_msg_srv.srv import GetBatteryState

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_service')
        self.sdk = create_standard_sdk("battery")
        self.robot = self.sdk.create_robot('192.168.80.3')
        self.robot.authenticate('user', 'wruzvkg4rce4')
        self.get_logger().info("ready to take commands")
        self.robot.time_sync.wait_for_sync()

        self._robot_id = self.robot.ensure_client(RobotIdClient.default_service_name).get_id(timeout=0.4)
        self._lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        self._power_client = self.robot.ensure_client(PowerClient.default_service_name)
        self._robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)

        #self.battery_publisher = self.create_publisher(Float32, '/battery', 10)
        self.srv = self.create_service(GetBatteryState, 'get_battery_state', self.get_battery_state_callback)

        #self.timer = self.create_timer(1.0, self.timer_callback)

    def get_battery_state_callback(self, request, response):
        state = self._robot_state_client.get_robot_state()
        battery_state = state.power_state.locomotion_charge_percentage.value
        self.get_logger().info(f"STATE: {battery_state}")
        response.battery_state = battery_state
        return response

    def timer_callback(self):
        state = self._robot_state_client.get_robot_state()
        battery_state = state.power_state.locomotion_charge_percentage.value
        self.get_logger().info(f"STATE: {battery_state}")

        msg = Float32()
        msg.data = battery_state
        self.battery_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("CTRL C")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

