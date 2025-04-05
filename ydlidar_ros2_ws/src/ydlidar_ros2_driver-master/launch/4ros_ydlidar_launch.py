#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get the package's share directory
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')

    # Declare the launch argument for the parameters file
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar_4ros.yaml'),
        description='Path to the ROS 2 parameters file to use.'
    )

    # Define the LifecycleNode for the driver
    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace='',
        output='screen',
        parameters=[parameter_file]
    )

    # Define the transform publisher node (if needed)
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser']
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        # Uncomment the line below if the transform publisher is required
        # tf2_node,
    ])
