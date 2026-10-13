"""Run the copied STD descriptor extraction utility."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = 'super_lio_slam'
    config = PathJoinSubstitution([FindPackageShare(package), 'config', 'std_extract.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/points_raw'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/std_descriptors'),
        DeclareLaunchArgument('max_visualized_descriptors', default_value='100'),
        Node(
            package=package,
            executable='std_extract_node',
            name='std_extract_node',
            output='screen',
            parameters=[config, {
                'input_topic': LaunchConfiguration('input_topic'),
                'output_dir': LaunchConfiguration('output_dir'),
                'max_visualized_descriptors': LaunchConfiguration('max_visualized_descriptors'),
            }],
        ),
    ])
