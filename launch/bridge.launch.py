from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fairino_bridge',
            executable='joint_state_bridge',
            output='screen'),
        Node(
            package='fairino_bridge',
            executable='follow_joint_trajectory_server',
            output='screen'),
    ])
