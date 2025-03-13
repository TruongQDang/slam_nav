import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
        package_name = 'slam_nav'
        pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')

        rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot_state_publisher.launch.py'
                )]), 
                launch_arguments={'use_sim_time': 'true'}.items()
        )

        # Gazebo Sim
        gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': '-r empty.sdf'}.items()
        )

        # RViz
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'tf_bridge.rviz')]
        )

        # Spawn
        spawn = Node(
                package='ros_gz_sim',
                executable='create',
                parameters=[{'name': 'my_robot',
                        'topic': 'robot_description'}],
                output='screen',
        )

        # Gz - ROS Bridge
        bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                # Clock (IGN -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # Joint states (IGN -> ROS2)
                '/world/empty/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/model/my_robot/pose@'
                'tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                ],
                remappings=[
                ('/model/my_robot/pose', '/tf'),
                ('/world/empty/model/my_robot/joint_state', 'joint_states'),
                ],
                output='screen'
        )


        # Launch them all!
        return LaunchDescription([
                rsp,
                gazebo,
                spawn,
                bridge,
                rviz
        ])

