import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    configuration_basename = 'cartographer_tb4_real.lua'

    use_sim = False

    return LaunchDescription([
        
        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            remappings=[('/odom', '/tb4_2/odom'),
                        ('/scan','/tb4_2/scan'),
                        ('/tf','/tb4_2/tf'),
                        ('/tf_static','/tb4_2/tf_static'),
                        ],
            parameters=[{'use_sim_time': use_sim}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            remappings=[('/odom', '/tb4_2/odom'),
                        ('/scan','/tb4_2/scan'),
                        ('/tf','/tb4_2/tf'),
                        ('/tf_static','/tb4_2/tf_static'),
                        ],
            parameters=[{'use_sim_time': use_sim}],
            arguments=['-resolution', '0.02', '-publish_period_sec', '1.0']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0.069", "0.002", "0.176", "0", "0", "0", "base_footprint", "tb4_2/rplidar_link"]
        ),
    ]) 