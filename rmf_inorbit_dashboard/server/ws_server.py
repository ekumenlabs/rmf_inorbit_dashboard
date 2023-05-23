#!/usr/bin/env python3

# Copyright 2023 InOrbit, Inc.

import argparse
import asyncio
import sys

import rclpy
import websockets
from websockets.exceptions import ConnectionClosedError

CONNECTIONS = set()


async def handle(websocket):
    """Handles a websocket connection
    Every message received is broadcasted to all active connections but the sender 

    Args:
        websocket (websockets.server.WebSocketServerProtocol)
    """

    CONNECTIONS.add(websocket)

    try:
        async for message in websocket:
            print("Message received")
            websockets.broadcast([c for c in CONNECTIONS if c != websocket], message)
    except ConnectionClosedError as e:
        print(f"ConnectionClosedError: {e}")

    CONNECTIONS.remove(websocket)


def main(argv=None):
    """Spin up the WS server

    Args:
        port (int): Port to listen in
        host (str | None, optional): Interface to listen in. Defaults to None.
    """

    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="InOrbit RMF dashboard backend",
        description="Configure and spin up the server")
    parser.add_argument("-p", "--port", type=int, required=False, default=8000,
                        help="Port for the server to listen on")
    parser.add_argument("-i", "--interface", type=str, required=False, default="localhost",
                        help="Empty will listen on all interfaces. Default: localhost")
    args = parser.parse_args(args_without_ros[1:])

    async def start(port: int, host=None):
        async with websockets.serve(handle, host, port):
            print(f"Websocket server listening at port :{port}")
            await asyncio.Future()  # run forever

    asyncio.run(start(host=args.interface, port=args.port))

if __name__ == "__main__":
    main(argv=sys.argv)
