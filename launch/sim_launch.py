import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():
        package_name = 'slam_nav' 

        # Robot state publisher
        robot_state_publisher = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(package_name),'launch','robot_state_publisher.launch.py'
                        )]))

        # Gazebo Sim
        world = os.path.join(
                get_package_share_directory(package_name),
                'worlds',
                'empty_world.world')    
        gazebo = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([os.path.join(
                                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                        launch_arguments={'gz_args': ['-r ', world]}.items())

        # Spawm robot
        spawn = Node(package='ros_gz_sim', executable='create',
                                parameters=[{
                                        'name': 'my_robot_sim',
                                        'topic': 'robot_description',
                                        'z': 0.5,}],
                                        output='screen')

        # Launch
        return LaunchDescription([
                robot_state_publisher,
                gazebo,
                spawn
        ])