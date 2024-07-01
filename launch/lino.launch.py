import os
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_path(package_name, subpaths=None):
    if subpaths:
        return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)
    else:
        return PathJoinSubstitution([FindPackageShare(package_name)])


def generate_launch_description():
    package_name = "my_autodock"

    # ref: https://answers.ros.org/question/396790/in-ros2-is-it-possible-to-set-gazebo_model_path-in-launch-file/
    pkg_install_path = get_package_share_directory(package_name)
    if "GAZEBO_RESOURCE_PATH" in os.environ:
        resource_path = pkg_install_path + ":" + os.environ["GAZEBO_RESOURCE_PATH"]
    else:
        resource_path = pkg_install_path

    use_sim_time = True
    # world_path = "/home/zealzel/my-gazebo-world/.sdf"
    world_path = get_path(package_name, ["worlds", "room_with_tags.sdf"])

    description_launch_path = get_path(
        "linorobot2_description", ["launch", "description.launch.py"]
    )
    gazebo_launch_path = get_path("gazebo_ros", ["launch", "gazebo.launch.py"])
    ekf_config_path = get_path("linorobot2_base", ["config", "ekf.yaml"])

    x_arg = DeclareLaunchArgument("x", default_value="0.5", description="x position")
    y_arg = DeclareLaunchArgument("y", default_value="0.5", description="y position")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_path,
        description="Gazebo world",
    )
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            "use_sim_time": str(use_sim_time),
            "publish_joints": "false",
        }.items(),
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            "world": LaunchConfiguration("world"),
        }.items(),
    )
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "zbot_lino",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
        ],
        output="screen",
    )
    command_timeout = Node(
        package="linorobot2_gazebo",
        executable="command_timeout.py",
        name="command_timeout",
    )
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, ekf_config_path],
        remappings=[("odometry/filtered", "odom")],
    )
    return LaunchDescription(
        [
            SetEnvironmentVariable(name="GAZEBO_RESOURCE_PATH", value=resource_path),
            # SetEnvironmentVariable(name="LINOROBOT2_BASE", value="zbotlino2"),
            SetEnvironmentVariable(name="LINOROBOT2_BASE", value="zbotlino2a"),
            world_arg,
            x_arg,
            y_arg,
            description,
            gazebo,
            spawn_entity,
            command_timeout,
            robot_localization,
        ]
    )
