"""Run Super-LIO + STD/ICP/PGO with a Unitree UniLiDAR L1."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('super_lio_slam')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([share, 'launch', 'slam.launch.py'])
            ),
            launch_arguments={
                'sensor_config': PathJoinSubstitution([share, 'config', 'unitree_l1.yaml']),
                'slam_config': PathJoinSubstitution([share, 'config', 'unitree_slam.yaml']),
                'rviz': LaunchConfiguration('rviz'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
