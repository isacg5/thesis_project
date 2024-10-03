from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
import subprocess
import signal
import os
import sys

def get_pid_with_command(command):
    result = subprocess.run(['ps', 'aux'], stdout=subprocess.PIPE, text=True)

    for line in result.stdout.splitlines():
        if command in line:
            columns = line.split()
            pid = int(columns[1])
            return pid
    return None

def handle_sigint(signal_received, frame):
    try:
        command = '/opt/ros/humble/lib/rclcpp_components/component_container_isolated'
        pid = get_pid_with_command(command)

        os.kill(pid, signal.SIGTERM)
    except OSError:
        print("Can't kill process")
    sys.exit(0)

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', ['/home/linuxbrew/ros2_ws/src/isabel_interface/scripts/vel2.py']],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['python3', ['/home/linuxbrew/ros2_ws/src/isabel_battery/scripts/pub_getbattery.py']],
            output='screen'
        ),
        TimerAction(
            period=7.0,  # Espera 1 segundo antes de lanzar el siguiente nodo
            actions=[
                Node(
                    package='isabel_interface',
                    executable='2cpp_executable',
                    name='cpp_node',
                    output='screen'
                )
            ]
        ),
        #Node(
        #    package='isabel_interface',
        #    executable='2cpp_executable',
        #    name='cpp_node',
        #    output='screen'
        #),
        TimerAction(
            period=10.0,  # Espera 2 segundos en total desde el inicio del lanzamiento antes de lanzar el siguiente nodo
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rtabmap_launcher.launch.py'])
                )
            ]
        )
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rtabmap_launcher.launch.py'])
#        )
    ])

signal.signal(signal.SIGINT, handle_sigint)
