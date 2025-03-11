import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    return LaunchDescription([
        # Launch gazebo
        ExecuteProcess(
            cmd=[
                'gz', 'sim', '-r',
                    '/home/truongdang/Documents/slam_nav_ws/src/slam_nav/description/moving_robot.sdf'
            ]
        ),
        # Launch a bridge to forward tf and joint states to ros2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/Moving_robot/model/vehicle_blue/joint_state@'
                'sensor_msgs/msg/JointState[gz.msgs.Model',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            remappings=[
                ('/world/Moving_robot/model/vehicle_blue/joint_state', '/joint_states'),
            ]
        ),
        # Launch rviz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'tf_bridge.rviz')]
        )
    ])