#!/usr/bin/env python3


from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace='turtlesim1', package='turtlesim',
            executable='turtlesim_node', output='screen'),
        launch_ros.actions.Node(
            namespace='kinematic_path', package='kinematic_path',
            executable='kinematic_path_node', output='screen'),
    ])

