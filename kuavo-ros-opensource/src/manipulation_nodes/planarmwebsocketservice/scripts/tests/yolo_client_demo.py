import asyncio
from time import sleep
import websockets
import json
import os
import rospkg
from datetime import datetime
import hashlib
import argparse

package_name = 'planarmwebsocketservice'
package_path = rospkg.RosPack().get_path(package_name)
PYTHON_FILE_FOLDER = package_path + "/scripts"
ACTION_FILE_FOLDER = package_path + "/action_files"    # 在运行之前确认文件的位置
MODEL_FOLDER = package_path + "/models"

# Expand the tilde to the full home directory path, 使用前请确认路径是否正确
action_file_name = "/hand_up_tag_right.tact"
action_file_path = ACTION_FILE_FOLDER + action_file_name
def calculate_file_md5(file_path: str) -> str:
    hash_md5 = hashlib.md5()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()
md5 = calculate_file_md5(action_file_path)

class WebSocketClient:
    def __init__(self, uri):
        self.uri = uri
        self.websocket = None

    async def connect(self):
        while True:
            try:
                self.websocket = await websockets.connect(self.uri)
                print("Connected to WebSocket server")
                return
            except Exception as e:
                print(f"Connection failed: {e}. Retrying in 5 seconds...")
                await asyncio.sleep(5)

    async def send_message(self, message):
        if not self.websocket:
            await self.connect()
        try:
            await self.websocket.send(json.dumps(message))
        except websockets.exceptions.ConnectionClosed:
            print("Connection closed. Reconnecting...")
            await self.connect()
            await self.send_message(message)

    async def receive_messages(self):
        if not self.websocket:
            await self.connect()
        try:
            response = await self.websocket.recv()
            print(f"Received response: {response}")
        except websockets.exceptions.ConnectionClosed:
            print("Connection closed. Reconnecting...")
            await self.connect()
            await self.receive_messages()

    async def close(self):
        if self.websocket:
            await self.websocket.close()
            self.websocket = None

async def send_test_message(uri):

    client = WebSocketClient(uri)

    try:
        await client.connect()

        path = "~/kuavo-ros-control/src/manipulation_nodes/planarmwebsocketservice/upload_files/test/main.py"
        message = {
            "cmd": "run_node",
            "data": {
                "path": path
            }
        }

        # message = {"cmd": "get_robot_info"}

        await client.send_message(message)
        print("Message sent, now listening for responses...")
        await client.receive_messages()
        print('------------------------------------')

        sleep(3)
        message = {"cmd": "get_robot_status"}
        await client.send_message(message)
        print("Message sent, now listening for responses...")
        await client.receive_messages()
        print('------------------------------------')

        sleep(3)
        message = {"cmd": "get_robot_status"}
        await client.send_message(message)
        print("Message sent, now listening for responses...")
        await client.receive_messages()
        print('------------------------------------')

        sleep(5)
        message = {"cmd": "stop_run_node"}
        # message = {"cmd": "stop_preview_action"}
        await client.send_message(message)
        print("Message sent, now listening for responses...")
        await client.receive_messages()
        print('------------------------------------')

        sleep(5)
        message = {"cmd": "get_robot_status"}
        await client.send_message(message)
        print("Message sent, now listening for responses...")
        await client.receive_messages()
        print('------------------------------------')

        # Continuously receive and print messages
        await client.receive_messages()

    except asyncio.CancelledError:
        print("Client was cancelled, closing connection.")
    finally:
        await client.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WebSocket Client for YOLO Demo")
    parser.add_argument("--uri", required=True, type=str, 
                        default="ws://{ip}:{port}",
                        help="WebSocket server URI (format: ws://{ip}:{port})")
    args = parser.parse_args()
    try:
        asyncio.run(send_test_message(args.uri))
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")
