"""Run Super-LIO and the online STD/ICP pose-graph backend."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = 'super_lio_slam'
    share = FindPackageShare(package)
    sensor_config = LaunchConfiguration('sensor_config')
    slam_config = LaunchConfiguration('slam_config')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_config',
            default_value=PathJoinSubstitution([share, 'config', 'livox_360.yaml']),
            description='Super-LIO sensor and frontend YAML file',
        ),
        DeclareLaunchArgument(
            'slam_config',
            default_value=PathJoinSubstitution([share, 'config', 'slam.yaml']),
            description='Keyframe, STD, ICP, and pose-graph YAML file',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(
            package=package,
            executable='super_lio_node',
            name='super_lio_node',
            output='screen',
            parameters=[sensor_config, slam_config, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package=package,
            executable='loop_backend_node',
            name='loop_backend_node',
            output='screen',
            parameters=[slam_config, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='super_lio_slam_rviz',
            arguments=['-d', PathJoinSubstitution([share, 'rviz', 'slam.rviz'])],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
