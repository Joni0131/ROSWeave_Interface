#!/usr/bin/env python3
"""
Launch file to preview `simple_car.xacro`:
- runs xacro to generate the robot_description
- starts robot_state_publisher with the generated URDF
- starts joint_state_publisher_gui
- starts rviz2

This script is intended to be run inside the container where `/usr/bin/xacro` and ROS2 are available.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = '/root/nav2_ws/urdf_modle/simple_car.xacro'
    
    robot_description = Command(['xacro ', xacro_file])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}]
    )

    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', '/root/nav2_ws/launch/robot_preview.rviz']
    )

    return LaunchDescription([
        rsp_node,
        jsp_node,
        rviz_node,
    ])


if __name__ == '__main__':
    # Allow running this file directly: it will act like `ros2 launch` for the contained description.
    import sys
    from launch import LaunchService

    ld = generate_launch_description()
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(ld)
    sys.exit(ls.run())
