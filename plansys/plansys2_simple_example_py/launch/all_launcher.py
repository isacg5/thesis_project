from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', ['/home/linuxbrew/ros2_ws/src/isabel_battery/scripts/getbattery.py']],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['python3', ['/home/linuxbrew/ros2_ws/src/zed/scripts/server.py']],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['python3', ['/home/linuxbrew/ros2_ws/src/plansys/plansys2_simple_example_py/plansys2_simple_example_py/gotopoint.py']],
            output='screen'
        ),
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/plansys2_launch.py'])
                )
            ]
        ),
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='plansys2_simple_example_py',
                    executable='controller_node',
                    name='controller_node',
                    output='screen'
                )
            ]
        ),

#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rtabmap_launcher.launch.py'])
#        )
    ])


