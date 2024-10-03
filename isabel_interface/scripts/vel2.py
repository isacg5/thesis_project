#EN LA PUBLICACION DE ODOM SI SE PONE BODY CON UNA Y A VECES VA Y A
#VECES NO EN LA VISUALIZACION DE RTABMAP, PERO SALE SIEMPRE EL ERROR DE TF
#!/usr/bin/env python3

from __future__ import print_function

from __future__ import absolute_import

import argparse
import collections
import logging
import sys
import time
import threading
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import rclpy
import math

import bosdyn
import bosdyn.client
import bosdyn.client.util

from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.frame_helpers import get_odom_tform_body

from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


#matplotlib.use('Qt5agg')
LOGGER = logging.getLogger(__name__)
TEXT_SIZE = 10
SPOT_YELLOW = '#FBD403'

pc2 = PointCloud2()
pub = None

def _update_thread(async_task):
    while True:
        async_task.update()
        time.sleep(0.01)

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

def send_transform(tf_broadcaster, odom_msg):
    t = TransformStamped()
    t.header.stamp = odom_msg.header.stamp
    t.header.frame_id = 'odom'
    t.child_frame_id = 'bodyy'

    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    #print(x,y)
    alpha = math.atan2(y, x)
    angles = euler_from_quaternion(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, -1 * odom_msg.pose.pose.orientation.z, -1* odom_msg.pose.pose.orientation.w)
    theta = angles[2]
    mod_p = math.sqrt(x**2 + y**2)

    t.transform.translation.x = mod_p * math.cos(alpha - theta) #odom_msg.pose.pose.position.x #-1 * odom_msg.pose.pose.position.x
    t.transform.translation.y = mod_p * math.sin(alpha - theta) #odom_msg.pose.pose.position.y #-1 * odom_msg.pose.pose.position.y
    t.transform.translation.z = 0.0 #odom_msg.pose.pose.position.z
    t.transform.rotation.x = odom_msg.pose.pose.orientation.x
    t.transform.rotation.y = odom_msg.pose.pose.orientation.y
    t.transform.rotation.z = odom_msg.pose.pose.orientation.z
    t.transform.rotation.w = -1*odom_msg.pose.pose.orientation.w

    tf_broadcaster.sendTransform(t)

def publish_data(node, robot):
    global pub, pc2
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    pub_odom = node.create_publisher(Odometry, 'odom', 1000)

    tf_broadcaster = TransformBroadcaster(node)
#    print(robot_state)
 #   print('Robot position: ', robot_position)
    freq = 30  #Hz
    while True:
        robot_state = robot_state_client.get_robot_state()
        robot_position = robot_state.kinematic_state.transforms_snapshot.child_to_parent_edge_map['odom'].parent_tform_child
        robot_v = robot_state.kinematic_state.velocity_of_body_in_odom
#        print(robot_state)
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = node.get_clock().now().to_msg()
        odom_msg.child_frame_id = "bodyy" # antes body, este funciona
#        odom_msg.child_frame_id = "body"
        odom_msg.pose.pose.position.x = -1 * robot_position.position.x
        odom_msg.pose.pose.position.y = -1 * robot_position.position.y
        odom_msg.pose.pose.position.z = robot_position.position.z
        odom_msg.pose.pose.orientation.x = robot_position.rotation.x
        odom_msg.pose.pose.orientation.y = robot_position.rotation.y
        odom_msg.pose.pose.orientation.z = robot_position.rotation.z
        odom_msg.pose.pose.orientation.w = robot_position.rotation.w
        odom_msg.twist.twist.linear.x = robot_v.linear.x
        odom_msg.twist.twist.linear.y = robot_v.linear.y
        odom_msg.twist.twist.linear.z = robot_v.linear.z
        odom_msg.twist.twist.angular.x = robot_v.angular.x
        odom_msg.twist.twist.angular.y = robot_v.angular.y
        odom_msg.twist.twist.angular.z = robot_v.angular.z

        #print("ROBOT: ", robot_position.position.x, robot_position.position.y, robot_position.rotation.x, robot_position.rotation.y, robot_position.rotation.z, robot_position.rotation.w)

#        print('publishhhhh')
        send_transform(tf_broadcaster, odom_msg)
        pub_odom.publish(odom_msg)
        pub.publish(pc2)
        time.sleep(1.0 / freq)

class AsyncPointCloud(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncPointCloud, self).__init__("point_clouds", robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_point_cloud_from_sources_async(["velodyne-point-cloud"])



class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__("robot_state", robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()


def window_closed(ax):
    fig = ax.figure.canvas.manager
    active_managers = plt._pylab_helpers.Gcf.figs.values()
    return not fig in active_managers


def set_axes_equal(ax):
    """Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Args
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])



def main():
    global pub, pc2
    # The last argument should be the IP address of the robot. The app will use the directory to find
    # the velodyne and start getting data from it.
    rclpy.init()
    node = rclpy.create_node('pc2_publisher')
    #pub = rospy.Publisher('points2', PointCloud2, queue_size=100)
    pub = node.create_publisher(PointCloud2, 'points2', 1000)

    '''parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(argv)'''

    sdk = bosdyn.client.create_standard_sdk('VelodyneClient')
    robot = sdk.create_robot('192.168.80.3')
    robot.authenticate('user', 'wruzvkg4rce4')
    #bosdyn.client.util.authenticate(robot)
    robot.sync_with_directory()

    _point_cloud_client = robot.ensure_client('velodyne-point-cloud')
    _robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    _point_cloud_task = AsyncPointCloud(_point_cloud_client)
    _robot_state_task = AsyncRobotState(_robot_state_client)
    _task_list = [_point_cloud_task, _robot_state_task]
    _async_tasks = AsyncTasks(_task_list)
    print('Connected.')

    ##########################################
    '''
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    pub_odom = node.create_publisher(Odometry, 'odom', 100)
    robot_state = robot_state_client.get_robot_state()
#    print(robot_state)
    robot_position = robot_state.kinematic_state.transforms_snapshot.child_to_parent_edge_map['odom'].parent_tform_child
    print('Robot position: ', robot_position)
    '''
    ##########################################

    publish_thread = threading.Thread(target=publish_data, args=[node, robot])
    publish_thread.daemon = True  # Set as daemon so it will be terminated when the main thread exits
    publish_thread.start()
    update_thread = threading.Thread(target=_update_thread, args=[_async_tasks])
    update_thread.daemon = True
    update_thread.start()

    # Wait for the first responses.
    while any(task.proto is None for task in _task_list):
        time.sleep(0.1)
    fig = plt.figure()


    # Plot the point cloud as an animation.
    ax = fig.add_subplot(111, projection='3d')
    aggregate_data = collections.deque(maxlen=1)
    #while not rospy.is_shutdown():
    
    while True:
        try:
#            pub = node.create_publisher(PointCloud2, 'points2', 10)
            if _point_cloud_task.proto[0].point_cloud:
                data = np.fromstring(_point_cloud_task.proto[0].point_cloud.data, dtype=np.float32)
                aggregate_data.append(data)
                plot_data = np.concatenate(aggregate_data)

                #print(len(plot_data))
                x=plot_data[0::3]
                y=plot_data[1::3]
                z=plot_data[2::3]
                i=(x*0)+1
#                print(len(x), " ", len(y), " ", len(z))
                #inserire qua il codice di github per convertire in pointcloud, il formato È float32.
                # HO 4 COSI DA INSERIRE: X,Y,Z,INTENSITà, PROBABILMENTE SE TOLGO L'ULTIMO SIAMO OK, PERCHè QUA LI LEGGOO
                #X,Y,Z SENZA INTENSITà APPARENTEMENTE
#                fields = [PointField('x', 0, PointField.FLOAT32, 1),
#                  PointField('y', 4, PointField.FLOAT32, 1),
#                  PointField('z', 8, PointField.FLOAT32, 1),
#                  PointField('intensity', 12, PointField.FLOAT32, 1),]
                fields = [
                   PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                   PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                   PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                   PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
                ]
                header = Header()
#                header.frame_id = 'sensors' # before was odom, este funciona
                header.frame_id = 'odom'
                #header.stamp = rospy.Time.now()
                header.stamp = node.get_clock().now().to_msg()
                points = np.array([x,y,z,i]).reshape(4,-1).T
                pc2 = point_cloud2.create_cloud(header, fields, points)
#                print(len(pc2.data))
                #pub.publish(pc2)




        except KeyboardInterrupt:
            print("CTRL C")
            exit()

if __name__ == '__main__':
    try:
        main()
    except rclpy.exceptions.ROSInterruptException:
        pass
