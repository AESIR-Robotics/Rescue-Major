#!/usr/bin/env python3
"""
WebRTC Server - Main Script
Handles HTTP server, routes, and coordinates ROS node with WebRTC manager.
"""
import argparse
import asyncio
import json
import logging
import os
import ssl
import threading
import time
import cv2
import numpy as np
import rclpy
import cv_bridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision.srv import Command
from aiohttp import web
from ament_index_python.packages import get_package_share_directory

from webrtc_manager import WebRTCManager

# Setting up logging and global variables
ROOT = os.path.dirname(__file__)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("pc")

# Global variables
intermediate_node = None
webrtc_manager = None


class Intermediate(Node):
    """
    ROS Node for handling image data and adjusting the quality based on network conditions.
    """
    def __init__(self, mode="auto"):
        super().__init__('intermediate_node')
        self.images_subscribers = []
        
        self.camera_topics = [
            "/cam0/image_raw",
            "/cam1/image_raw",
            "/cam2/image_raw"
        ]
        self.sensor_topic = "/cam_sensors/image"
        
        self.active_sensor_track = None
        self.pending_sensor_track_switch = None  # Pending track switch request
        
        self.camera_count = len(self.camera_topics)
        self.latest_images = [None] * self.camera_count
        self.bridge = cv_bridge.CvBridge()
        self.last_time = time.time()
        self.fps = 30
        self.lock = threading.Lock()
        self.subscription_lock = threading.Lock()  # Protect subscription operations
        
        # Load placeholder image from package share if available, otherwise fall back
        try:
            placeholder_path = os.path.join(get_package_share_directory('teleoperation'), 'GUI', 'placeholder.jpg')
        except Exception:
            placeholder_path = os.path.normpath(os.path.join(ROOT, '..', 'share', 'teleoperation', 'GUI', 'placeholder.jpg'))

        if os.path.exists(placeholder_path):
            self.placeholder_image = cv2.imread(placeholder_path)
            if self.placeholder_image is None:
                logger.warning('Found placeholder at %s but cv2 failed to read it', placeholder_path)
                self.placeholder_image = np.zeros((480, 640, 3), dtype=np.uint8)
        else:
            logger.warning('Placeholder image not found at %s; using blank image', placeholder_path)
            self.placeholder_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        self.new_image = None
        self.rtt = None
        self.last_rtt = None
        self.manual_resolution = (1881, 1051)
        self.resolution = (1920, 1080)
        self.mode = mode

        self.preallocate_latest_images()

        # Initial subscription to cameras
        self._subscribe_to_cameras()
        
        # Timer to process pending track switches safely in main thread
        self.create_timer(0.05, self._process_pending_switch)

        # Add service for WebRTC commands
        self.webrtc_command_service = self.create_service(
            Command,
            "/web_rtc_commands",
            self.webrtc_command_callback
        )

        logger.info("Intermediate Node initialized in %s mode", self.mode)

    def image_callback(self, msg, index):
        """Callback function to process the incoming image messages."""
        current_time = time.time()
        if current_time - self.last_time >= 1.0 / self.fps:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                if self.mode == "manual":
                    resized_image = cv2.resize(cv_image, self.manual_resolution)
                else:
                    resized_image = self.resize_image(cv_image)

                # Thread-safe buffer update
                with self.subscription_lock:
                    # Check if buffer needs reallocation due to size change
                    if self.latest_images[index].shape != resized_image.shape:
                        logger.info(f"Track {index}: Reallocating buffer from {self.latest_images[index].shape} to {resized_image.shape}")
                        self.latest_images[index] = np.zeros(resized_image.shape, dtype=np.uint8)
                    
                    np.copyto(self.latest_images[index], resized_image)
                
                self.last_time = current_time
            except Exception as e:
                logger.error(f"Error in image_callback for track {index}: {e}")

    def webrtc_command_callback(self, request, response):
        """
        Service callback to process WebRTC commands.
        """
        command = request.data
        logger.info(f"Received WebRTC command: {command}")
    
        if command.startswith("vision:camera,"):
            # switch track N to sensors
            try:
                track_index = int(command.split(',')[1])
                
                # Validate before setting flag
                if track_index < 0 or track_index >= self.camera_count:
                    logger.error(f"Invalid track index: {track_index}. Must be 0-{self.camera_count-1}")
                    response.success = False
                    response.message = f"Invalid track index: {track_index}"
                else:
                    logger.info(f"Request to switch track {track_index} to {'raw' if self.active_sensor_track == track_index else 'sensors'}")
                    
                    # Set pending switch flag - will be processed by timer in main thread
                    self.pending_sensor_track_switch = track_index
                    
                    response.success = True
                    response.message = f"Track {track_index} switch requested"
            except (ValueError, IndexError) as e:
                logger.error(f"Invalid vision:camera command format: {command}. Error: {e}")
                response.success = False
                response.message = f"Invalid command format: {e}"
        
        elif command == "audio:mute":
            try:
                webrtc_manager.mute_audio()
                response.success = True
                response.message = "Audio muted"
            except Exception as e:
                logger.error(f"Error muting audio: {e}")
                response.success = False
                response.message = f"Failed to mute audio: {e}"
        
        elif command == "audio:unmute":
            try:
                webrtc_manager.unmute_audio()
                response.success = True
                response.message = "Audio unmuted"
            except Exception as e:
                logger.error(f"Error unmuting audio: {e}")
                response.success = False
                response.message = f"Failed to unmute audio: {e}"
        
        else:
            logger.warning(f"Unknown WebRTC command: {command}")
            response.success = False
            response.message = f"Unknown command: {command}"
        
        return response
    
    def _process_pending_switch(self):
        """Timer callback to process pending track switches in main ROS thread."""
        if self.pending_sensor_track_switch is not None:
            track_index = self.pending_sensor_track_switch
            self.pending_sensor_track_switch = None  # Clear flag
            logger.info(f"Processing track {track_index} switch in main thread")
            self._switch_sensor_track(track_index)
    
    def _subscribe_to_cameras(self):
        """Initialize all camera subscriptions. Thread-safe."""
        with self.subscription_lock:
            for sub in self.images_subscribers:
                self.destroy_subscription(sub)
            self.images_subscribers.clear()
            
            for i in range(self.camera_count):
                topic = self.sensor_topic if i == self.active_sensor_track else self.camera_topics[i]
                logger.info(f"Track {i} subscribing to: {topic} {'(SENSORS)' if i == self.active_sensor_track else ''}")
                
                # Use functools.partial to avoid closure issues
                from functools import partial
                callback = partial(self.image_callback, index=i)
                
                sub = self.create_subscription(
                    Image,
                    topic,
                    callback,
                    10
                )
                self.images_subscribers.append(sub)
                
                # Clear image buffer for this track to avoid showing stale data
                self.latest_images[i] = np.zeros(self.resolution[::-1] + (3,), dtype=np.uint8)
    
    def _switch_sensor_track(self, track_index):
        """Switch a camera track between raw and sensor feeds."""
        if track_index is not None and (track_index < 0 or track_index >= self.camera_count):
            logger.error(f"Invalid track index: {track_index}. Must be between 0 and {self.camera_count-1}")
            return
        
        with self.subscription_lock:
            # Toggle: if already on sensors, switch back to raw
            if self.active_sensor_track == track_index:
                logger.info(f"Track {track_index} toggling back to raw feed")
                old_sensor_track = self.active_sensor_track
                self.active_sensor_track = None  # No track reads sensors
            else:
                old_sensor_track = self.active_sensor_track
                self.active_sensor_track = track_index
                logger.info(f"Track {track_index} switching to sensors feed")
            
            # Determine which tracks need updating
            tracks_to_update = set()
            if old_sensor_track is not None:
                tracks_to_update.add(old_sensor_track)
            if track_index is not None:
                tracks_to_update.add(track_index)
            
            # Update only affected subscriptions
            from functools import partial
            for i in tracks_to_update:
                if i >= len(self.images_subscribers):
                    logger.warning(f"Track {i} subscription index out of range")
                    continue
                
                # Destroy previous subscription
                try:
                    self.destroy_subscription(self.images_subscribers[i])
                except Exception as e:
                    logger.error(f"Error destroying subscription for track {i}: {e}")
                
                # Determine new topic
                topic = self.sensor_topic if i == self.active_sensor_track else self.camera_topics[i]
                logger.info(f"Track {i} switching to: {topic} {'(SENSORS)' if i == self.active_sensor_track else '(RAW)'}")
                
                # Create new subscription with proper closure handling
                callback = partial(self.image_callback, index=i)
                
                try:
                    sub = self.create_subscription(
                        Image,
                        topic,
                        callback,
                        10
                    )
                    self.images_subscribers[i] = sub
                    
                    # Clear image buffer to avoid showing stale frames from previous source
                    self.latest_images[i] = np.zeros(self.resolution[::-1] + (3,), dtype=np.uint8)
                    logger.info(f"Track {i} buffer cleared")
                    
                except Exception as e:
                    logger.error(f"Error creating subscription for track {i}: {e}")

    def update_bandwidth(self, msg):
        self.bandwidth = msg.data
        self.adjust_fps_and_resolution()
    
    def preallocate_latest_images(self):
        width, height = self.resolution
        shape = (height, width, 3)
        for i in range(self.camera_count):
            self.latest_images[i] = np.zeros(shape, dtype=np.uint8)

    def adjust_fps_and_resolution(self):
        rtt_settings = {
            (0, 3): {'resolution': (1920, 1080), 'fps': 30},
            (4, 7): {'resolution': (820, 720), 'fps': 30},
            (8, 11): {'resolution': (640, 480), 'fps': 15},
            (12, float('inf')): {'resolution': (320, 240), 'fps': 5},
        }
        for (lower_bound, upper_bound), settings in rtt_settings.items():
            if lower_bound <= self.rtt < upper_bound:
                self.fps = settings['fps']
                self.resolution = settings['resolution']
                logger.info("Adjusted FPS to %s and resolution to %s", self.fps, self.resolution)
                break
        self.preallocate_latest_images()

    def resize_image(self, image):
        if self.rtt is not None:
            if self.rtt is not self.last_rtt:
                self.adjust_fps_and_resolution()
                self.last_rtt = self.rtt
            return cv2.resize(image, self.resolution)
        return image

    def get_latest_image(self, index):
        """Returns the latest processed image or a placeholder if none available."""
        return self.latest_images[index] if self.latest_images[index] is not None else self.placeholder_image


# Web handler functions
async def index(request):
    """Serves the index.html page."""
    content = open(os.path.join(ROOT, "../GUI/index.html"), "r").read()
    return web.Response(content_type="text/html", text=content)


async def offer(request):
    """Handles the WebRTC offer from the client."""
    return await webrtc_manager.handle_offer(request, intermediate_node)


async def on_shutdown(app):
    await webrtc_manager.shutdown()


def main(args=None):
    """Main entry point for the WebRTC server."""
    parser = argparse.ArgumentParser(
        description="WebRTC video streaming server"
    )
    parser.add_argument("--cert-file", help="SSL certificate file (for HTTPS)")
    parser.add_argument("--key-file", help="SSL key file (for HTTPS)")
    parser.add_argument(
        "--host", default="0.0.0.0", help="Host for HTTP server (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--port", type=int, default=8081, help="Port for HTTP server (default: 8081)"
    )
    parser.add_argument("--verbose", "-v", action="count")
    parser.add_argument(
        "--mode", 
        choices=["manual", "auto"], 
        default="auto", 
        help="Set mode to 'manual' or 'auto' (default: 'auto')"
    )

    args, unknown = parser.parse_known_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    if args.cert_file:
        ssl_context = ssl.SSLContext()
        ssl_context.load_cert_chain(args.cert_file, args.key_file)
    else:
        ssl_context = None

    # Initialize WebRTC manager
    global webrtc_manager
    webrtc_manager = WebRTCManager()

    # App initialization
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    gui_path = os.path.join(ROOT, "../GUI")
    app.router.add_static("/static/", gui_path)
    app.router.add_get("/", index)
    app.router.add_post("/offer", offer)

    # ROS node creation
    rclpy.init()
    global intermediate_node
    intermediate_node = Intermediate(mode=args.mode)

    # Start the ROS node in a separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(intermediate_node), daemon=True)
    ros_thread.start()

    # Start the web application
    web.run_app(
        app, access_log=None, host=args.host, port=args.port, ssl_context=ssl_context
    )

    ros_thread.join()


if __name__ == "__main__":
    main()