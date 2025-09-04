from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_following_behavior',
            executable='wall_follower_exec',
            output='screen'),
    ])