from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('std_detector'), 'config', 'std_extract.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/points_raw'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/std_descriptors'),
        DeclareLaunchArgument('max_visualized_descriptors', default_value='100'),
        Node(package='std_detector', executable='std_extract_node',
             name='std_extract_node', output='screen',
             parameters=[config, {'input_topic': LaunchConfiguration('input_topic'),
                                  'output_dir': LaunchConfiguration('output_dir'),
                                  'max_visualized_descriptors': ParameterValue(
                                      LaunchConfiguration('max_visualized_descriptors'), value_type=int)}]),
    ])
