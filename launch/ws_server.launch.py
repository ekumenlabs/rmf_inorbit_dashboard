#!/usr/bin/env python3

# Copyright 2023 InOrbit, Inc.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    port_arg = DeclareLaunchArgument(
        name="ws_port",
        default_value="8000",
        description="Websocket server port",
    )
    port = LaunchConfiguration('ws_port')

    interface_arg = DeclareLaunchArgument(
        name="ws_interface",
        default_value="localhost",
        description="Interface for the websocket server to listen in",
    )
    interface = LaunchConfiguration('ws_interface')

    # Launch server
    server_node = Node(
        package='rmf_inorbit_dashboard',
        executable='ws_server',
        arguments=["--port", port, "--interface", interface],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        port_arg,
        interface_arg,
        server_node,
    ])
