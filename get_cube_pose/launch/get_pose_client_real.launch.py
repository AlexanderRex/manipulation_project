import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_grasping',
            executable='basic_grasping_perception_real_node',
            name='basic_grasping_perception',
            parameters=[{'debug_topics': True}],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='get_cube_pose',
            executable='get_pose_client',
            name='get_pose_client',
            output='screen',
            emulate_tty=True,
        )
    ])
