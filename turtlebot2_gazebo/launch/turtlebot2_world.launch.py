import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    tutlebot2_gazebo_package = FindPackageShare(
        package="turtlebot2_gazebo").find("turtlebot2_gazebo")

    turtlebot2_description_package = FindPackageShare(
        package="turtlebot2_description").find("turtlebot2_description")

    gazebo_ros_package = FindPackageShare(
        package="gazebo_ros").find("gazebo_ros")

    world = os.path.join(
        tutlebot2_gazebo_package, "worlds", "test.world.model"
    )

    # install_dir = get_package_prefix(turtlebot2_description_package)
    # gazebo_models_path = os.path.join(tutlebot2_gazebo_package, "models")

    # # x_pose = LaunchConfiguration("x_pose", default="0.0")
    # # y_pose = LaunchConfiguration("y_pose", default="0.0")

    # if "GAZEBO_MODEL_PATH" in os.environ:
    #     os.environ["GAZEBO_MODEL_PATH"] = (
    #         os.environ["GAZEBO_MODEL_PATH"]
    #         + ":"
    #         + install_dir
    #         + "/share"
    #         + ":"
    #         + gazebo_models_path
    #     )
    # else:
    #     os.environ["GAZEBO_MODEL_PATH"] = (
    #         install_dir + "/share" + ":" + gazebo_models_path
    #     )

    # if "GAZEBO_PLUGIN_PATH" in os.environ:
    #     os.environ["GAZEBO_PLUGIN_PATH"] = (
    #         os.environ["GAZEBO_PLUGIN_PATH"] + ":" + install_dir + "/lib"
    #     )
    # else:
    #     os.environ["GAZEBO_PLUGIN_PATH"] = install_dir + "/lib"

    # print("GAZEBO MODELS PATH==" + str(os.environ["GAZEBO_MODEL_PATH"]))
    # print("GAZEBO PLUGINS PATH==" + str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # ============================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_package, "launch", "gazebo.launch.py"),
        )
    )

    # start_gazebo_client_cmd = ExecuteProcess(cmd=["gzclient"], output="screen")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        os.path.join(
                            turtlebot2_description_package,
                            "robots/kobuki_hexagons_hokuyo.urdf.xacro",
                        ),
                    ]
                )
            },
            {"use_sim_time": True},
        ],
    )

    spawn_tb2 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        output="screen",
        arguments=[
            "-entity",
            "two_wheeled_robot",
            # '-timeout','300',
            "-file",
            os.path.join(turtlebot2_description_package,"robots","tmp.urdf"),
            "-x",
            "0",
            "-y",
            "0",
        ],
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(
                tutlebot2_gazebo_package, 'worlds', 'test.world.model'), ''],
            description='SDF world file')])
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_tb2)

    return ld
