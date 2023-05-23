#!/usr/bin/env python3

# Copyright 2023 InOrbit, Inc.

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Launches the REST API server and the websocket server at once"""

    # Include launches
    rest_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                "rmf_inorbit_dashboard"), "rest_server.launch.py")
        )
    )
    ws_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("rmf_inorbit_dashboard"), "ws_server.launch.py")),
    )

    return LaunchDescription([
        rest_launch_include,
        ws_launch_include
    ])
