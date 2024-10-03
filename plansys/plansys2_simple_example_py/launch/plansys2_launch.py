# Copyright 2019 Intelligent Robotics Lab
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_simple_example_py')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/domain.pddl', #my_example.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    move_cmd = Node(
        package='plansys2_simple_example_py',
        executable='move_action_node_fake.py',
        name='move_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    search_cmd = Node(
        package='plansys2_simple_example_py',
        executable='search_action_node_fake.py',
        name='search_action_node_track',
        namespace=namespace,
        output='screen',
        parameters=[])

    evaluate_cmd = Node(
        package='plansys2_simple_example_py',
        executable='evaluate_action_node.py',
        name='evaluate_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    check_cmd = Node(
        package='plansys2_simple_example_py',
        executable='check_action_node_fake.py',
        name='check_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    dialog_cmd = Node(
        package='plansys2_simple_example_py',
        executable='dialog_action_node.py',
        name='dialog_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    report_cmd = Node(
        package='plansys2_simple_example_py',
        executable='report_action_node_fake.py',
        name='report_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(move_cmd)
    ld.add_action(search_cmd)
    ld.add_action(check_cmd)
    ld.add_action(evaluate_cmd)
    ld.add_action(dialog_cmd)
    ld.add_action(report_cmd)

    return ld
