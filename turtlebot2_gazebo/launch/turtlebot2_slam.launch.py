import os
from unicodedata import name

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch.actions import TimerAction


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params_file = LaunchConfiguration('slam_params_file')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    turtlebot2_gazebo_dir = get_package_share_directory('turtlebot2_gazebo')

    turtlebot2_world_launch = os.path.join(get_package_share_directory(
        'turtlebot2_gazebo'), 'launch', 'turtlebot2_world.launch.py')

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to apply a namespace to the navigation stack'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot2_world_launch]),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'use_namespace': use_namespace}.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ('/goal_pose', 'goal_pose'),
                        ('/clicked_point', 'clicked_point'),
                        ('/initialpose', 'initialpose')]),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(get_package_share_directory("slam_toolbox"),
                                       'config', 'mapper_params_online_async.yaml'),
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),

        Node(
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        )
    ])
