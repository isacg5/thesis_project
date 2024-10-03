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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import math
from custom_msg_srv.msg import Trajectory
import time

class MoveAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('move', 0.5)

        self.waypoints = {}
        self.initialize_waypoints()

        self.progress_ = 0.0
        self.position_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
#        self.position_publisher = self.create_publisher(Trajectory, '/goaltospot', 10)

        self.subscription = self.create_subscription(
            ActionExecution,
            '/actions_hub', self.listener_callback, 10)

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_pos = Pose()

        self.current_goal = None
        self.wp = None
        self.location = None

    def odom_callback(self, msg):
#        print("ODOOOOOOOOOOOOOM", msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.current_pos.position.x = msg.pose.pose.position.x
        self.current_pos.position.y = msg.pose.pose.position.y
        self.current_pos.position.z = msg.pose.pose.position.z
        self.current_pos.orientation.x = msg.pose.pose.orientation.x
        self.current_pos.orientation.y = msg.pose.pose.orientation.y
        self.current_pos.orientation.z = msg.pose.pose.orientation.z
        self.current_pos.orientation.w = msg.pose.pose.orientation.w


    def listener_callback(self, msg):
        parameters = msg.arguments
        action_name = msg.action
        self.get_logger().info(f'Action received with parameters {parameters}')

        # Here, perform any action-specific handling based on action_name and parameters
        if action_name == 'move':
            if len(parameters) == 3:  # Expected 'move <from> <to>'
                start_point = parameters[1]
                end_point = parameters[2]
                self.get_logger().info(f'Moving from {start_point} to {end_point}')
                self.wp = end_point
                self.location = parameters[2]

    def initialize_waypoints(self):
        positions = [
            ("a", (0.0, 0.0)),
            ("b", (0.0, 1.5)), #("b", (-1.0, 0.5)), #0.7 1.64 para world
            ("c", (-0.6, 1.0)),
            ("battery_point", (0.0, 0.0)),
            ("exit", (0.0, 0.0))
        ]

        for name, (x,y) in positions:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'odom'
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            self.waypoints[name] = pose_msg


    def do_work(self):
        if self.progress_ < 0.25:
            self.progress_ += 0.05
        else:
            if self.current_goal is None:
                if self.wp in self.waypoints:
                    self.current_goal = self.waypoints[self.wp]
                    self.send_navigation_goal(self.current_goal)
                else:
                    self.get_logger().error(f"Waypoint {self.wp} not defined.")

            else:
                distance, ang_dif = self.get_distance()
                print("DISSSSSSSSSSSSSSST", distance, 'ang dif ', ang_dif)
                if(distance < 0.1 and ang_dif < 0.2):
                    self.get_logger().info("Move completed")
                    #time.sleep(5)
                    self.send_feedback(self.progress_, "Moved " + self.location)
                    time.sleep(1)
                    self.finish(True, 1.0, "Finished move")
                    self.current_goal = None  # Reset for next goal
                    self.progress_ = 0.0
                else:
                    self.send_feedback(0.0, 'Move running')


    def send_navigation_goal(self, goal_pos):
        self.position_publisher.publish(goal_pos)
        self.get_logger().info(f"Sending goal to {goal_pos.pose.position.x}, {goal_pos.pose.position.y}")


    def quaternion_angle_diff(self, q1, q2):
        dot_product = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w
        dot_product = max(min(dot_product, 1.0), -1.0)
        return 2 * math.acos(dot_product)

    def get_distance(self):
        dx = self.current_goal.pose.position.x - self.current_pos.position.x
        dy = self.current_goal.pose.position.y - self.current_pos.position.y
        distance = (dx**2 + dy**2)**0.5
        angular_difference = self.quaternion_angle_diff(self.current_pos.orientation, self.current_goal.pose.orientation)
        print("GOAL: ", self.current_goal.pose.position.x, self.current_goal.pose.position.y, "CURRENT: ", self.current_pos.position.x, self.current_pos.position.y)
        return distance, angular_difference

    '''
    def do_work(self):
        self.publish_position()
        if self.progress_ < 1.0:
            self.progress_ += 0.05
            self.send_feedback(self.progress_, 'Move running')
        else:
            self.finish(True, 1.0, 'Move completed');
            self.progress_ = 0.0
        
        self.get_logger().info('Moving ... {}'.format(self.progress_))
   

    def publish_position(self):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = 0.7
        pose_msg.pose.position.y = 1.64
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.position_publisher.publish(pose_msg)
    '''

def main(args=None):
    rclpy.init(args=args)

    node = MoveAction()
    node.set_parameters([Parameter(name='action_name', value='move')])

    node.trigger_configure()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

