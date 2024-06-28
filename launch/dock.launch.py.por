import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "my_autodock"

    detect_pose_launch_dir = os.path.join(
        get_package_share_directory(package_name), "launch"
    )

    dock_params_dir = os.path.join(get_package_share_directory(package_name), "params")
    params_file = os.path.join(dock_params_dir, "my_docking.yaml")

    dock_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                detect_pose_launch_dir,
                "detect.launch.py",
            )
        )
    )
    docking_server = Node(
        package="opennav_docking",
        executable="opennav_docking",
        name="docking_server",
        output="screen",
        parameters=[params_file],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_docking",
        output="screen",
        parameters=[{"autostart": True}, {"node_names": ["docking_server"]}],
    )
    return LaunchDescription(
        [
            # robot_base_launch,
            dock_detection_launch,
            docking_server,
            lifecycle_manager,
        ]
    )
