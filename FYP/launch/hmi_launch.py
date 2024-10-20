# launch/hmi_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Path to the backend directory
    backend_path = os.path.join(os.getcwd(), 'backend')

    return LaunchDescription([
        # Launch the FastAPI backend using uvicorn
        ExecuteProcess(
            cmd=['uvicorn', 'main:app', '--host', '0.0.0.0', '--port', '8000', '--reload'],
            cwd=backend_path,
            name='fastapi',
            output='screen'
        ),
        # Launch the ROS2 Interface Node
        Node(
            package='ros_hmi_project,  # Replace with your package name
            executable='ros_interface.py',    # Replace with your node executable name
            name='ros2_interface',
            output='screen'
        )
    ])
