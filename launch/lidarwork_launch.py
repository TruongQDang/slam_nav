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
                os.path.join(
                    pkg_ros_gz_sim_demos,
                    'models',
                    '/home/truongdang/Documents/slam_nav_ws/src/slam_nav/description/slam_world.sdf'
                )
            ]
        ),
        # Launch a bridge to forward tf and joint states to ros2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/gpu_lidar_sensor/model/vehicle_blue/joint_state@'
                'sensor_msgs/msg/JointState[gz.msgs.Model',
                '/model/vehicle_blue/pose@'
                'tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/tf_foo@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                'lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            ],
            remappings=[
                ('/model/vehicle_blue/pose', '/tf'),
                ('/world/gpu_lidar_sensor/model/vehicle_blue/joint_state', '/joint_states'),
                ('/tf_foo', '/tf')
            ]
        ),
        # Launch rviz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/home/truongdang/Documents/slam_nav_ws/src/slam_nav/config/lidar_config.rviz']
        )
    ])