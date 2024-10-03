from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', ['/home/linuxbrew/ros2_ws/src/isabel_interface/scripts/vel2.py']],
            output='screen'
        ),
        #ExecuteProcess(
        #    cmd=['python3', ['/home/linuxbrew/ros2_ws/src/isabel_battery/scripts/pub_getbattery.py']],
        #    output='screen'
        #),
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

