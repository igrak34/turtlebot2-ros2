from argparse import Namespace
from distutils.cmd import Command
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from launch_ros.substitutions import FindPackageShare
import launch
import yaml


def generate_launch_description():
    use_namespace=LaunchConfiguration('use_namespace')
    namespace=LaunchConfiguration('namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')
    turtlebot2_nav_dir = get_package_share_directory('turtlebot2_nav')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='tb2_6',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_namespace',
            default_value='true',
            description='Whether to apply a namespace to the navigation stack'),

        DeclareLaunchArgument(  # TODO! automatic substitution namespace for proper topics
            'rviz_config',
            default_value=os.path.join(
                turtlebot2_nav_dir,
                'rviz', 
                'namespaced_tb2_6.rviz'),
            description='Full path to the RVIZ config file to use'),
        
        DeclareLaunchArgument(
            'use_namespace',
            default_value='true',
            description='Whether to apply a namespace to the navigation stack'),
        
        Node(
            package='rviz2',
            executable='rviz2',
            namespace=namespace,
            arguments=['-d', rviz_config_file],
            output='screen',
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ('/goal_pose', 'goal_pose'),
                        ('/clicked_point', 'clicked_point'),
                        ('/initialpose', 'initialpose')]),

        
    ])