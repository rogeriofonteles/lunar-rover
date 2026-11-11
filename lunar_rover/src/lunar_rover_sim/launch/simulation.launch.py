#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_lunar_rover_sim = get_package_share_directory('lunar_rover_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths - using Perseverance SDF model (works with Gazebo Harmonic)
    perseverance_model = os.path.join(pkg_lunar_rover_sim, 'urdf', 'perseverance', 'model.sdf')
    world_file = os.path.join(pkg_lunar_rover_sim, 'worlds', 'mars_world.sdf')

    # Set GZ_SIM_RESOURCE_PATH so Gazebo can find models and textures
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_lunar_rover_sim, 'urdf') + ':' +
            os.path.join(pkg_lunar_rover_sim, 'worlds')
        ]
    )

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items()
    )

    # Spawn Perseverance rover on top of terrain
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'perseverance',
            '-file', perseverance_model,
            '-x', '0',
            '-y', '0',
            '-z', '5.0',
        ],
        output='screen'
    )

    # ROS-Gazebo bridge for topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        gz_resource_path,
        gz_sim,
        spawn_robot,
        bridge,
    ])
