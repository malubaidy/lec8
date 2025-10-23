import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Package and file setup
    pkg_name = 'urdf_example'
    xacro_file = os.path.join(
        get_package_share_directory(pkg_name),
        'description',
        'example_robot.urdf.xacro'
    )

    # Process the xacro file to generate robot description
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Launch argument for simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Launch Gazebo (Harmonic / Ignition)
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),  # empty world
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time
        }]
    )

    # Spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'my_bot',
            '-topic', 'robot_description',
            '-z', '0.5'
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock if true'),
        gz_launch,
        robot_state_publisher,
        spawn_entity
    ])
