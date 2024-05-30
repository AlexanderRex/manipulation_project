import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    action_server_node = Node(
            package='simple_grasping',
            executable='basic_grasping_perception_real_node',
            name='basic_grasping_perception',
            parameters=[{'debug_topics': True}],
            output='screen',
            emulate_tty=True,
    )
    
    pick_and_place_node = Node(
            name="pick_and_place_perception_real",
            package="moveit2_scripts",
            executable="pick_and_place_perception_real",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {'use_sim_time': False},
        ],
    )
    
    return LaunchDescription([
        action_server_node,
        pick_and_place_node
    ])
