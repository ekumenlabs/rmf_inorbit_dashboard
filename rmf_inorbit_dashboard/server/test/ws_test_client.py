#!/usr/bin/env python3

# Copyright 2023 InOrbit, Inc.

import argparse
import asyncio

import websockets
import sys


# https://websockets.readthedocs.io/en/stable/howto/patterns.html#consumer-and-producer
async def test_client(uri):
    async with websockets.connect(uri) as websocket:
        speaker = asyncio.create_task(prompt(websocket))
        listener = asyncio.create_task(listen_everything(websocket))
        done, pending = await asyncio.wait(
            [listener, speaker],
            return_when=asyncio.FIRST_COMPLETED,
        )
        for task in pending:
            task.cancel()


async def listen_everything(websocket):
    while True:
        msg = await websocket.recv()
        print(f"\n<<< {msg}")


async def prompt(websocket):
    while True:
        message = await ainput("Write something: ")
        await websocket.send(message)


# https://stackoverflow.com/a/65326191
async def ainput(string: str) -> str:
    await asyncio.get_event_loop().run_in_executor(
        None, lambda s=string: sys.stdout.write(s+' '))
    return await asyncio.get_event_loop().run_in_executor(
        None, sys.stdin.readline)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Prints everything the server sends to it")
    parser.add_argument("-u", "--uri", type=str, required=False, default="ws://localhost:8000",
                        help="Server URI. Default: \"ws://localhost:8000\"")
    args = parser.parse_args()
    asyncio.run(test_client(args.uri))
