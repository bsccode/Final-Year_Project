#/backend/ros_interface.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from threading import Lock
import base64
import cv2
from cv_bridge import CvBridge
import json

class ROS2Interface(Node):
    def __init__(self):
        super().__init__('ros2_interface')
        self.bridge = CvBridge()
        
        # Separate locks for camera and map data
        self.camera_lock = Lock()
        self.map_lock = Lock()

        # Initialize the data store
        self.data_store = {
            "camera_color_image_raw": b"",
            "map": {},
            "robot_position": {"x": 0, "y": 0}  # Store robot position here
        }

        # Dictionary to keep track of subscribers: {topic_name: subscriber}
        self.subscribers = {}

        self.get_logger().info("ROS2Interface initialized.")
        self.subscribe_to_camera()
        self.subscribe_to_map()

    def subscribe_to_camera(self):
        """ Automatically subscribe to the camera feed topic """
        topic_name = '/camera/color/image_raw'
        topic_type = 'sensor_msgs/msg/Image'
        self.subscribe_topic(topic_name, topic_type)

    def subscribe_to_map(self):
        """ Automatically subscribe to the map topic """
        topic_name = '/map'
        topic_type = 'nav_msgs/msg/OccupancyGrid'
        self.subscribe_topic(topic_name, topic_type)

    def subscribe_topic(self, topic_name, topic_type):
        """ Subscribe to a specific ROS topic """
        if topic_type == "sensor_msgs/msg/Image":
            subscriber = self.create_subscription(
                Image,
                topic_name,
                self.image_callback,
                10
            )
            self.get_logger().info(f"Subscribed to {topic_name}")
            self.subscribers[topic_name] = subscriber

        elif topic_type == "nav_msgs/msg/OccupancyGrid":
            subscriber = self.create_subscription(
                OccupancyGrid,
                topic_name,
                self.map_callback,
                10
            )
            self.get_logger().info(f"Subscribed to {topic_name}")
            self.subscribers[topic_name] = subscriber

        else:
            self.get_logger().warning(f"Unsupported topic type: {topic_type}")

    def unsubscribe_topic(self, topic_name):
        """ Unsubscribe from a specific ROS topic """
        if topic_name in self.subscribers:
            self.subscribers[topic_name].destroy()
            del self.subscribers[topic_name]
            self.get_logger().info(f"Unsubscribed from {topic_name}")
        else:
            self.get_logger().info(f"Not subscribed to {topic_name}")

    def image_callback(self, msg):
        try:
            if msg.encoding == 'bgr8':
                # Convert ROS Image message to OpenCV image (color image)
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                _, buffer = cv2.imencode('.jpg', cv_image)
                with self.camera_lock:
                    self.data_store["camera_color_image_raw"] = buffer.tobytes()

            elif msg.encoding == 'rgb8':
                # Convert ROS Image message to OpenCV image (RGB image)
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                # OpenCV uses BGR format, so we need to convert RGB to BGR
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                _, buffer = cv2.imencode('.jpg', cv_image)
                with self.camera_lock:
                    self.data_store["camera_color_image_raw"] = buffer.tobytes()

            elif msg.encoding == '32FC1':
                # Convert depth image to OpenCV format
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                _, buffer = cv2.imencode('.jpg', depth_colormap)
                with self.camera_lock:
                    self.data_store["camera_color_image_raw"] = buffer.tobytes()

            else:
                self.get_logger().warning(f"Unsupported image encoding: {msg.encoding}")

        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")

    def map_callback(self, msg):
        """ Callback for processing incoming map (OccupancyGrid) data """
        try:
            resolution = msg.info.resolution
            width = msg.info.width
            height = msg.info.height
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            data = msg.data

            # Initialize previous map if not present
            if "previous_map" not in self.data_store:
                self.data_store["previous_map"] = [-1] * (width * height)

            # Compare the new map with the previous map
            updates = []
            for y in range(height):
                row_data = data[y * width:(y + 1) * width]
                prev_row_data = self.data_store["previous_map"][y * width:(y + 1) * width]

                # Check if this row has changed
                if row_data != prev_row_data:
                    updates.append({"row": y, "data": list(row_data)})

            # Store the new map as the previous map for the next update
            self.data_store["previous_map"] = list(data)

            # Store map metadata and the updates
            with self.map_lock:
                self.data_store["map"] = {
                    "resolution": resolution,
                    "width": width,
                    "height": height,
                    "origin": {"x": origin_x, "y": origin_y},
                    "updates": updates  # Only the updated rows
                }
        except Exception as e:
            self.get_logger().error(f"Map callback error: {e}")

    async def get_camera_data(self):
        """ Get the latest camera feed data """
        with self.camera_lock:
            return self.data_store["camera_color_image_raw"]

    async def get_map_data(self):
        """ Get the latest map (SLAM) data """
        with self.map_lock:
            return self.data_store["map"]

    async def list_topics(self):
        """ List available ROS topics """
        topic_names_and_types = self.get_topic_names_and_types()
        supported_topics = [
            {"name": name, "type": types[0]} 
            for name, types in topic_names_and_types
        ]
        return supported_topics

def ros_spin(ros_interface):
    """ Keep ROS spinning """
    rclpy.spin(ros_interface)
    ros_interface.destroy_node()
    rclpy.shutdown()
