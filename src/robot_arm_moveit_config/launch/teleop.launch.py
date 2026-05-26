import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_arm").to_moveit_configs()

    servo_yaml = os.path.join(
        get_package_share_directory('arm_control'),
        'config',
        'servo_config.yaml'
    )

    tracker = Node(
        package='arm_control',
        executable='tracker_node',
        output='screen',
        parameters=[moveit_config.robot_description, {'use_sim_time': True}]
    )
    
    translator = Node(
        package='arm_control',
        executable='translator_node',
        output='screen',
        parameters=[moveit_config.robot_description, {'use_sim_time': True}]
    )

    return LaunchDescription([
        tracker,
        translator
 ])