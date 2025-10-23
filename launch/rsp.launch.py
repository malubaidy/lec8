import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get package path and xacro file
    pkg_path = os.path.join(get_package_share_directory('urdf_example'))
    xacro_file = os.path.join(pkg_path, 'description', 'example_robot.urdf.xacro')

    # Process the xacro file
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Create nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node
    ])
