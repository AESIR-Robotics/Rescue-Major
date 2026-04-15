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
from functools import partial
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
            "/oak/rgb/image_raw",
            "/cam1/image_raw",
        ]
        
        self.source_topics = {
            "raw": self.camera_topics,
            "sensors": "/cam_sensors/image",
            "thermal": "/cam_thermal/image_raw"
        }
        
        self.camera_count = len(self.camera_topics)
        
        # All tracks start showing "raw" by default
        self.track_sources = ["raw"] * self.camera_count
        self.pending_track_switches = []  # List of tuples: (track_index, target_source)
        
        self.latest_images = [None] * self.camera_count
        self.bridge = cv_bridge.CvBridge()
        self.last_times = [time.time()] * self.camera_count
        self.fps = 30
        self.lock = threading.Lock()
        self.subscription_lock = threading.Lock()  # Protect subscription operations
        
        # Load placeholder image from package share if available, otherwise fall back
        try:
            placeholder_path = os.path.join(get_package_share_directory('teleoperation'), 'GUI', 'assets/placeholder.jpg')
        except Exception:
            placeholder_path = os.path.normpath(os.path.join(ROOT, '..', 'share', 'teleoperation', 'GUI', 'assets', 'placeholder.jpg'))

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
        self.manual_resolution = (1920, 1080)
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
        if current_time - self.last_times[index] < (1.0 / self.fps) and self.fps <= 15:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            if self.mode == "manual":
                resized_image = self.limit_resolution(cv_image, self.manual_resolution)
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
    
        if command.startswith("vision:"):
            try:
                parts = command.split(",")
                cmd_type = parts[0].split(":")[1]  # "camera", "sensors", or "thermal"
                track_index = int(parts[1])

                if track_index < 0 or track_index >= self.camera_count:
                    logger.error(f"Invalid track index: {track_index}")
                    response.success = False
                    response.message = f"Invalid track index: {track_index}"
                    return response

                # Determine the target source type toggle
                current_source = self.track_sources[track_index]
                
                # Toggle logic: if in raw, go to requested mode; otherwise go back to raw
                if current_source == "raw":
                    target_source = "thermal" if cmd_type == "thermal" else "sensors"
                else:
                    target_source = "raw"

                logger.info(f"Request to switch track {track_index} to {target_source}")
                self.pending_track_switches.append((track_index, target_source))

                response.success = True
                response.message = f"Track {track_index} switch requested to {target_source}"

            except (ValueError, IndexError) as e:
                logger.error(f"Invalid vision command format: {command}. Error: {e}")
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
        if self.pending_track_switches:
            for track_index, target_source in list(self.pending_track_switches):
                logger.info(f"Processing track {track_index} switch to {target_source} in main thread")
                self._switch_track_source(track_index, target_source)
            self.pending_track_switches.clear()
    
    def _get_topic_for_track(self, index, source):
        """Helper to resolve the topic string based on source mode."""
        if source == "raw":
            return self.source_topics["raw"][index]
        return self.source_topics[source]

    def _clear_image_buffer(self, index):
        """Re-initializes the image buffer to black to drop stale frames."""
        self.latest_images[index] = np.zeros(self.resolution[::-1] + (3,), dtype=np.uint8)

    def _subscribe_to_cameras(self):
        """Initialize all camera subscriptions. Thread-safe."""
        with self.subscription_lock:
            for sub in self.images_subscribers:
                self.destroy_subscription(sub)
            self.images_subscribers.clear()
            
            for i in range(self.camera_count):
                source = self.track_sources[i]
                topic = self._get_topic_for_track(i, source)
                    
                logger.info(f"Track {i} subscribing to: {topic} ({source.upper()})")
                
                callback = partial(self.image_callback, index=i)
                
                sub = self.create_subscription(
                    Image,
                    topic,
                    callback,
                    10
                )
                self.images_subscribers.append(sub)
                self._clear_image_buffer(i)
    
    def _switch_track_source(self, track_index, new_source):
        """Switch a camera track to a new source (raw, sensors, thermal)."""
        if track_index is not None and (track_index < 0 or track_index >= self.camera_count):
            logger.error(f"Invalid track index: {track_index}. Must be between 0 and {self.camera_count-1}")
            return
            
        if new_source not in self.source_topics:
            logger.error(f"Invalid source: {new_source}. Must be one of {list(self.source_topics.keys())}")
            return
        
        with self.subscription_lock:
            old_source = self.track_sources[track_index]
            if old_source == new_source:
                logger.info(f"Track {track_index} is already on {new_source}, ignoring")
                return
                
            self.track_sources[track_index] = new_source
            logger.info(f"Track {track_index} switched from {old_source} to {new_source}")
            
            # Destroy and recreate subscription
            if track_index < len(self.images_subscribers):
                try:
                    self.destroy_subscription(self.images_subscribers[track_index])
                except Exception as e:
                    logger.error(f"Error destroying subscription for track {track_index}: {e}")
            else:
                logger.warning(f"Track {track_index} subscription index out of range")
                return
                
            topic = self._get_topic_for_track(track_index, new_source)
            logger.info(f"Track {track_index} new subscription: {topic}")
            
            callback = partial(self.image_callback, index=track_index)
            
            try:
                sub = self.create_subscription(
                    Image,
                    topic,
                    callback,
                    10
                )
                self.images_subscribers[track_index] = sub
                self._clear_image_buffer(track_index)
                logger.info(f"Track {track_index} buffer cleared")
                
            except Exception as e:
                logger.error(f"Error creating subscription for track {track_index}: {e}")

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
            (0, 40): {'resolution': (1920, 1080), 'fps': 30},
            (40, 80): {'resolution': (1280, 720), 'fps': 30},
            (80, 150): {'resolution': (720, 480), 'fps': 30},
            (150, 250): {'resolution': (640, 480), 'fps': 15},
            (250, float('inf')): {'resolution': (320, 240), 'fps': 5},
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
            return self.limit_resolution(image, self.resolution)
        return image
    
    def limit_resolution(self, image, max_resolution):
        """
        Reduce la imagen solo si excede el límite de resolución, 
        manteniendo la relación de aspecto y asegurando dimensiones pares para el color en WebRTC.
        """
        target_w, target_h = max_resolution
        h, w = image.shape[:2]
        
        # Solo aplicamos cv2.resize si la imagen es MÁS GRANDE que nuestro límite
        if w > target_w or h > target_h:
            # Calcular factor de escala para mantener la proporción (aspect ratio)
            scale = min(target_w / w, target_h / h)
            new_w = int(w * scale)
            new_h = int(h * scale)
            
            # CRÍTICO: Asegurar que ancho y alto sean PARES para evitar perder el color en YUV420p
            new_w -= new_w % 2
            new_h -= new_h % 2
            
            return cv2.resize(image, (new_w, new_h))
            
        # Si es más pequeña (como la térmica) o igual, se envía en su tamaño original intacto
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