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
from launch.conditions import IfCondition, UnlessCondition


def get_path(package_name, subpaths=None):
    if subpaths:
        return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)
    else:
        return PathJoinSubstitution([FindPackageShare(package_name)])


def generate_launch_description():
    package_name = "my_autodock"
    use_sim_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Enable use_sime_time to true",
    )
    use_rviz_arg = DeclareLaunchArgument(
        name="rviz", default_value="true", description="Run rviz"
    )
    # rviz_config_path = "/home/zealzel/.rviz2/nav2_camera.rviz"
    rviz_config_path = get_path(package_name, ["rviz", "nav2_camera.rviz"])
    default_map_path = get_path(package_name, ["maps", "room_with_tags.yaml"])
    map_arg = DeclareLaunchArgument(
        name="map",
        default_value=default_map_path,
        description="Navigation map path",
        # condition=UnlessCondition(LaunchConfiguration("sim")),
    )
    navigation_path = get_path(
        "linorobot2_navigation", ["launch", "navigation.launch.py"]
    )
    lino_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_path),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rviz_config": rviz_config_path,
            "map": LaunchConfiguration("map"),
        }.items(),
    )
    dock_path = get_path(package_name, ["launch", "dock.launch.py"])
    dock_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dock_path),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    return LaunchDescription(
        [
            use_sim_arg,
            use_rviz_arg,
            map_arg,
            lino_navigation,
            dock_robot,
        ]
    )
