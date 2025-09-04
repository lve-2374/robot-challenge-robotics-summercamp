from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ],
            arguments=['-d', '/home/user/ros2_ws/src/rviz/tb4_cartographer.rviz'],
            remappings=[
                ('/tf', '/tb4_2/tf'),
                ('/tf_static', '/tb4_2/tf_static')
            ]
        )
    ])
