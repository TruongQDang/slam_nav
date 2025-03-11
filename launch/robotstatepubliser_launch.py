import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        # Package name and Path to xacro file
        pkg_name = 'slam_nav'
        file_subpath = 'description/robot_urdf.xacro'

        # Parse robot description from xacro
        xacro_file = os.path.join(
                get_package_share_directory(pkg_name),
                file_subpath
        )
        robot_description = xacro.process_file(xacro_file).toxml()

        # Robot state publisher
        node_robot_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name="robot_state_publisher",
                output='both',
                parameters=[{'robot_description': robot_description}]
        )

        # Run the node
        return LaunchDescription([
                node_robot_state_publisher
        ])