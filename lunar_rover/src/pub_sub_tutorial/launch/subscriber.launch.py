from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='pub_sub_tutorial',
      executable='subscriber_node',
      name='rogerio_subscriber',
      output='screen',
    )
  ])