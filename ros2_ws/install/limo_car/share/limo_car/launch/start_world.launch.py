import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'limo_car'
    world_file_path = 'worlds/empty_world.model'
    rviz_path = 'rviz/gazebo.rviz'
    pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    ### Robot description ###
    pkg_robot_description = get_package_share_directory('limo_description')
    xacro_file_name = "limo_four_diff.xacro"
    xacro_file = os.path.join(
        pkg_robot_description,
        "urdf", xacro_file_name
    )
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()


    pkg_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(pkg_path, world_file_path)

    pkg_limo_car = get_package_share_directory(package_name)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(pkg_limo_car, 'rviz', 'gazebo.rviz')
        ]
    )

    # Robot state publisher
    params = {'use_sim_time': True, 'robot_description': robot_desc}
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[params],
            arguments=[])

    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'

    map_package = get_package_share_directory("limo_description")
    world_file = PathJoinSubstitution([map_package, "worlds", "husarion_world.sdf"])
    world_cfg = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world", default_value=["-r ", world_file], description="SDF world file"
    )
    ### GAZEBO IGNITION ### 
    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args': world_cfg}.items(),
    # )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -r -v 3 empty.sdf"}.items(),
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("limo_description"),
            "config",
            "limo_drive_controller.yaml",
        ]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "limo",
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            spawn_x_val,
            "-y",
            spawn_y_val,
            "-z",
            spawn_z_val,
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["limo_diff_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription([
        #declare_world_arg,
        #gz_sim,
        gazebo,
        control_node,
        robot_state_publisher,
        #rviz_node,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ])
