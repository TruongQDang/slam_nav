import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
        package_name = 'slam_nav'

        image_publisher = Node(
                package=package_name,
                executable='image_publisher',
                name='image_publisher'
        )

        vo_node = Node(
                package=package_name,
                executable='vo_node',
                name='vo_node'
        )

        rviz = Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(get_package_share_directory(package_name), 'config', 'odom_config.rviz')],
                output='screen'
        )

        return LaunchDescription([
                vo_node,
                rviz,                
                image_publisher
        ])