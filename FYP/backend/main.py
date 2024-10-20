#/backend/main.py

import os
import pty
import subprocess
import asyncio
import rclpy
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from ros_interface import ros_spin, ROS2Interface
import threading
import json
import logging
import base64
import zlib
import base64
from io import BytesIO

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Initialize ROS2
rclpy.init()

# Initialize ROS2Interface node
ros_interface = ROS2Interface()

# Start ROS2 spinning in a separate thread
ros_thread = threading.Thread(target=ros_spin, args=(ros_interface,), daemon=True)
ros_thread.start()

# Logging setup
logging.basicConfig(level=logging.INFO)

# Launch ttyd terminal server in the background
def launch_ttyd():
    try:
        home_directory = os.path.expanduser("~")  # Get the home directory path
        subprocess.Popen(['ttyd', '--writable', '-p', '8080', 'bash', '-c', f'cd {home_directory} && bash'])
        print("ttyd launched successfully on port 8080")
    except Exception as e:
        print(f"Error launching ttyd: {e}")

launch_ttyd()

@app.websocket("/ws/terminal")
async def terminal_websocket(websocket: WebSocket):
    await websocket.accept()
    pid, fd = pty.fork()

    if pid == 0:
        os.execv("/bin/bash", ["/bin/bash"])
    else:
        try:
            while True:
                output = os.read(fd, 1024).decode('utf-8')
                await websocket.send_text(output)
                data = await websocket.receive_text()
                os.write(fd, data.encode('utf-8'))
        except WebSocketDisconnect:
            print("Client disconnected")
        finally:
            await websocket.close()

@app.websocket("/ws/camera_feed")
async def websocket_camera_feed(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            with ros_interface.camera_lock:
                frame_data = ros_interface.data_store.get("camera_color_image_raw", None)
                if frame_data:
                    await websocket.send_text(base64.b64encode(frame_data).decode('utf-8'))  # Ensure base64 encoded
            await asyncio.sleep(0.05)  # 10 FPS
    except Exception as e:
        print(f"WebSocket error (camera feed): {e}")
    finally:
        if not websocket.client_state.CLOSING and not websocket.client_state.CLOSED:
            await websocket.close()
        print("Camera WebSocket closed")

@app.websocket("/ws/topics")
async def websocket_topics(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            topics = await ros_interface.list_topics()
            await websocket.send_json({"topics": topics})
            await asyncio.sleep(10)  # Send every 5 seconds
    except Exception as e:
        print(f"WebSocket error (topics): {e}")
    finally:
        await websocket.close()

@app.websocket("/ws/map_data")
async def websocket_map_data(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            with ros_interface.map_lock:
                map_data = ros_interface.data_store.get("map", {})
                updates = map_data.get("updates", [])

                if updates:
                    # Send map data along with robot position
                    await websocket.send_json({
                        "resolution": map_data["resolution"],
                        "width": map_data["width"],
                        "height": map_data["height"],
                        "origin": map_data["origin"],
                        "updateRegions": updates,  # Send only updated rows
                    })
            await asyncio.sleep(5)  # Update SLAM map every second
    except Exception as e:
        print(f"WebSocket error (map data): {e}")
    finally:
        # Correct WebSocket state handling
        await websocket.close()
        print("SLAM Map WebSocket closed")



if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
