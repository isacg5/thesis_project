#!/usr/bin/env python3
import logging
import math
import signal
import sys
import threading
import time
from sys import platform
import numpy as np
import rclpy 
import math
from math import atan2, asin 
import bosdyn.client
import bosdyn.client.util
from bosdyn import geometry
from bosdyn.api import geometry_pb2, image_pb2, trajectory_pb2, world_object_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b,
                                         get_vision_tform_body, ODOM_FRAME_NAME)
from bosdyn.client.lease import LeaseClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_id import RobotIdClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.world_object import WorldObjectClient

#from isabel_exploration.msg import Trajectory
from custom_msg_srv.msg import Trajectory
from custom_msg_srv.msg import Rotate
from rclpy.exceptions import ROSInterruptException
from bosdyn.geometry import EulerZXY
from std_msgs.msg import Float32

LOGGER = logging.getLogger()
_robot_command_client=None
_robot=None
_powered_on=False

t_x = 0
t_y = 0

def goal_clbk(goal):
    global mobility_params, _robot_command_client, t_x, t_y
    orientation= euler_from_quaternion(goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w)
    mobility_params= set_mobility_params()
    tag_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=goal.target_pose.pose.position.x, goal_y=goal.target_pose.pose.position.y,
                goal_heading=orientation[2], frame_name=ODOM_FRAME_NAME, params=mobility_params,
                body_height=0.0, locomotion_hint=spot_command_pb2.HINT_AUTO)
    end_time = 10.0
    #rospy.loginfo("command ready")
    node.get_logger().info("command ready")
    t_x = goal.target_pose.pose.position.x
    t_y = goal.target_pose.pose.position.y 
    print(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w)
    _robot_command_client.robot_command(lease=None, command=tag_cmd,end_time_secs=time.time() + end_time)


def set_mobility_params():
    obstacles = spot_command_pb2.ObstacleParams(disable_vision_body_obstacle_avoidance=False,
                                                disable_vision_foot_obstacle_avoidance=False,
                                                disable_vision_foot_constraint_avoidance=False,
                                                disable_vision_foot_obstacle_body_assist= False,
                                                disable_vision_negative_obstacles=False,
                                                obstacle_avoidance_padding=0.1)

    footprint_R_body = geometry.EulerZXY()
    position = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
    rotation = footprint_R_body.to_quaternion()
    pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
    point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
    traj = trajectory_pb2.SE3Trajectory(points=[point])
    body_control=spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

    speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(linear=Vec2(x=1.0, y=1.0), angular=0.7))        
    mobility_params = spot_command_pb2.MobilityParams( obstacle_params=obstacles, vel_limit=speed_limit, body_control=body_control, locomotion_hint=spot_command_pb2.HINT_AUTO)
    return mobility_params

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z # in radians


def rot_clbk(goal):
    global mobility_params, _robot_command_client, _robot_state_client, t_x, t_y
    print("AAAA")
    rad = math.radians(goal.rotation.data)
    print("RADIANS", rad)
    robot_state = _robot_state_client.get_robot_state()
    robot_position = robot_state.kinematic_state.transforms_snapshot.child_to_parent_edge_map['odom'].parent_tform_child
    print("R POSITION: ", t_x, " ", t_y)

    #orientation= euler_from_quaternion(goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w)
    #print("TRUE", orentation[2])  
    mobility_params= set_mobility_params()
    tag_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=t_x, goal_y=t_y,
                goal_heading=rad, frame_name=ODOM_FRAME_NAME, params=mobility_params,
                body_height=0.0, locomotion_hint=spot_command_pb2.HINT_AUTO)
    end_time = 10.0
    #rospy.loginfo("command ready")
    node.get_logger().info("command ready")
    #print(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w)
    _robot_command_client.robot_command(lease=None, command=tag_cmd,end_time_secs=time.time() + end_time)


def approach_clbk(goal):
    global mobility_params, _robot_command_client, _robot_state_client, t_x, t_y, rad
    print("R POSITION: ", t_x, " ", t_y, " rad: ", rad)
    deep = goal.data
    # check deep and add to the front axis (idk if it's x or y) goal_x = t_x + deep
    # VERIFICAR CUAL ES EL QUE ANDA HACIA DELANTE
    mobility_params= set_mobility_params()
    tag_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=t_x, goal_y=t_y,
                goal_heading=rad, frame_name=ODOM_FRAME_NAME, params=mobility_params,
                body_height=0.0, locomotion_hint=spot_command_pb2.HINT_AUTO)
    end_time = 10.0
    node.get_logger().info("command ready")

    _robot_command_client.robot_command(lease=None, command=tag_cmd,end_time_secs=time.time() + end_time)


def power_on():
    global _robot, _powered_on
    _robot.power_on()
    _powered_on=True

def power_off():
    safe_power_off_cmd=RobotCommandBuilder.safe_power_off_command()
    _robot_command_client.robot_command(command= safe_power_off_cmd)
    time.sleep(2.5)
    _powered_on=False


def node():
    global node, goal, mobility_params, _robot, _robot_id, _power_client, _robot_state_client , _robot_command_client
    rclpy.init()
    node = rclpy.create_node('timeretrieve')

    sdk = create_standard_sdk("gotopoint")

    robot = sdk.create_robot('192.168.80.3')
    robot.authenticate('user', 'wruzvkg4rce4')
    
#    rospy.Subscriber("goaltospot", TrajectoryActionGoal, goal_clbk)    
    node.create_subscription(Rotate, "/rotation", rot_clbk, 10)
    node.create_subscription(Trajectory, "goaltospot", goal_clbk, 10)
    node.create_subscription(Float32, "/approach", approach_clbk, 10)
    node.get_logger().info("ready swedrfgbto take commands")
    robot.time_sync.wait_for_sync()

   # node.get_logger().info("ready swedrfgbto take commands")

    _robot = robot
    _robot_id = robot.ensure_client(RobotIdClient.default_service_name).get_id(timeout=0.4)
    _lease_client = robot.ensure_client(LeaseClient.default_service_name)
    _power_client = robot.ensure_client(PowerClient.default_service_name)
    _robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    _robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    
    mobility_params= set_mobility_params()
    _lease = _lease_client.take()
    _lease_keepalive = bosdyn.client.lease.LeaseKeepAlive(_lease_client)
    power_on()
    time.sleep(5)
    blocking_stand(_robot_command_client)
    time.sleep(5)
    node.get_logger().info("ready to take commands")
    '''
    footprint_R_body = EulerZXY(yaw=0.4, roll=0.0, pitch=0.0)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    _robot_command_client.robot_command(cmd)
    #cmd = RobotCommandBuilder.synchro_stand_command(body_height=0.1)
    #_robot_command_client.robot_command(cmd)
    power_off()
    exit()
    '''
    #while not rospy.is_shutdown():
    while True:
        try:
            time.sleep(2.5)
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("CTRL C")
            power_off()
            exit()    

def main(args=None):
    try:
        node()
    except rclpy.exceptions.ROSInterruptException:
        pass 

if __name__ == '__main__':

    main()
