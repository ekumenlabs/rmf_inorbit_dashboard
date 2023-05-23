#!/usr/bin/env python3

# Copyright 2023 InOrbit, Inc.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

import rmf_inorbit_dashboard


def generate_launch_description():

    # Launch arguments
    port_arg = DeclareLaunchArgument(
        name="rest_port",
        default_value="8001",
        description="Port at which the server will expose the REST API",
    )
    port = LaunchConfiguration('rest_port')

    # Get location of the folder where the python executable is
    package_location = os.path.join(
        os.path.dirname(rmf_inorbit_dashboard.__file__))

    # Serve using uvicorn
    cmd = ['python3', '-m', 'uvicorn', '--app-dir',
           package_location, 'server.rest_server:app',
           '--port', port]

    process = ExecuteProcess(
        cmd=cmd,
        shell=True,
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        process,
    ])
