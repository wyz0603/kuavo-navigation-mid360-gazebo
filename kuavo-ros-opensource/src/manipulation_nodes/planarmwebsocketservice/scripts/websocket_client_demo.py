import asyncio
import websockets
import json
import os
from utils import calculate_file_md5

# # Expand the tilde to the full home directory path, 使用前请确认路径是否正确
# action_file_path = os.path.expanduser("/home/user/vvrrbag_2025-04-14-18-28-33_output.tact")
# # action_file_path = os.path.expanduser("/home/user/kuavo-ros-control/src/manipulation_nodes/planarmwebsocketservice/action_files/hand_up_tag_right.tact")
# md5 = calculate_file_md5(action_file_path)

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
            while True:
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

async def send_test_message(action_file_path):
    uri = "ws://0.0.0.0:8888"  # Adjust the URI as needed
    client = WebSocketClient(uri)

    try:
        await client.connect()

        # Expand the tilde to the full home directory path, 使用前请确认路径是否正确
        md5 = calculate_file_md5(action_file_path)
        # action_filename 发给 server, 要和 action_file_path 一致
        action_filename = os.path.basename(action_file_path)
        message = {
            "cmd": "preview_action",
            "data": {"action_filename": action_filename,
                     "action_file_MD5": md5},
        }
        await client.send_message(message)
        print("Message sent, now listening for responses...")

        # Continuously receive and print messages
        await client.receive_messages()

    except asyncio.CancelledError:
        print("Client was cancelled, closing connection.")
    finally:
        await client.close()

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='WebSocket client for sending action files')
    parser.add_argument('--action_file_path', type=str, required=True,
                      help='Path to the action file to be sent')
    args = parser.parse_args()
    try:
        asyncio.run(send_test_message(args.action_file_path))
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")
