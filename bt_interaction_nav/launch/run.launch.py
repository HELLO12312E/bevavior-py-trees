"""Launch file for bt_interaction_nav bt_runner_node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("bt_interaction_nav")
    params_file = os.path.join(pkg_share, "config", "params.yaml")

    bt_runner = Node(
        package="bt_interaction_nav",
        executable="bt_runner_node",
        name="bt_runner_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file],
    )

    return LaunchDescription([bt_runner])
