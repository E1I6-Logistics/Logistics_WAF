"""Publish the keepout mask and metadata required by waffle_real_nav.yaml."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_mask = Path(__file__).resolve().parents[1] / 'map/logitle_map_fin_keepout.yaml'
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument(
            'mask', default_value=str(default_mask),
            description='Absolute path to the keepout mask YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='nav2_map_server', executable='map_server',
            name='keepout_mask_server', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': ParameterValue(LaunchConfiguration('mask'), value_type=str),
                'topic_name': '/keepout_filter_mask',
                'frame_id': 'map',
            }]),
        Node(
            package='nav2_map_server', executable='costmap_filter_info_server',
            name='costmap_filter_info_server', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'type': 0,
                'filter_info_topic': '/costmap_filter_info',
                'mask_topic': '/keepout_filter_mask',
                'base': 0.0,
                'multiplier': 1.0,
            }]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_keepout', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['keepout_mask_server', 'costmap_filter_info_server'],
            }]),
    ])
