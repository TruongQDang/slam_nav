import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():
        package_name = 'slam_nav' 

        gazebo = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([os.path.join(
                                get_package_share_directory(package_name), 'launch', 'gz.launch.py')])
        )

        rviz = Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(get_package_share_directory(package_name), 'config', 'nav_config.rviz')],
                output='screen'
        )

        map = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
                        launch_arguments={'slam_params_file': './src/slam_nav/config/mapper_params_online_async.yaml'}.items())
        
        nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]),
                        launch_arguments={'use_sim_time': 'true'}.items()) 

        return LaunchDescription([
                gazebo,
                rviz,
                map,
                nav2
        ])