#!/usr/bin/env python3
"""
Launch file to preview `simple_car.xacro`:
- runs xacro to generate the robot_description
- starts robot_state_publisher with the generated URDF
- starts joint_state_publisher_gui
- starts rviz2

This script is intended to be run inside the container where `/usr/bin/xacro` and ROS2 are available.
"""

from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import argparse

def generate_launch_description(file_format=None):

    if file_format == 'xacro':
        default_model_path = '/root/nav2_ws/urdf_modle/simple_car.xacro'
    elif file_format == 'sdf':
        default_model_path = '/root/nav2_ws/urdf_modle/simple_car.sdf'
    else:
        raise ValueError("Unsupported file format")

    default_rviz_config_path = '/root/nav2_ws/launch/robot_preview.rviz'

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])

if __name__ == '__main__':
    # Allow running this file directly: it will act like `ros2 launch` for the contained description.
    import sys
    from launch import LaunchService
    parser = argparse.ArgumentParser(description="Example flag parser")
    parser.add_argument('--file_format', type=str, help='Type of the file to process (xacro or sdf)')

    args = parser.parse_args()
    file_format = args.file_format

    ld = generate_launch_description(file_format=file_format)
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(ld)
    sys.exit(ls.run())
