#!/usr/bin/env python3
# Importing necessary libraries
import argparse
import asyncio
import json
import logging
import os
import ssl
import uuid
import time
import cv2
import threading
import rclpy
import cv_bridge 
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int16MultiArray
from aiohttp import web
from av import VideoFrame, AudioFrame
from fractions import Fraction
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
import numpy as np
import subprocess
import signal
from av import VideoFrame, AudioFrame
import av

from audio_reproducer import AudioReproducer

# Setting up logging and global variables
ROOT = os.path.dirname(__file__)

# Configure logging early so it's available for ROS callbacks
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("pc")
pcs = set()


class Intermediate(Node):
    """
    ROS Node for handling image data and adjusting the quality based on network conditions.
    """
    def __init__(self, mode="auto"):
        super().__init__('intermediate_node')
        self.images_subscribers = []
        #camera_topic = ["/cam0/image_raw", "/cam1/image_raw"]
        camera_topic = ["/cam0/image_raw", "/cam1/image_raw", "/cam2/image_raw"]
        self.camera_count = len(camera_topic)
        self.latest_images = [None] * self.camera_count
        self.bridge = cv_bridge.CvBridge()
        self.last_time = time.time()
        self.fps = 30
        self.lock = threading.Lock()
        # Load placeholder image from package share if available, otherwise fall back
        try:
            from ament_index_python.packages import get_package_share_directory
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

        # Audio management
        self.audio_tracks = set()
        self.active_peers = set()
        
        # Store main event loop reference for cross-thread calls
        self.main_loop = None

        # AudioReproducer instance for direct playback
        self.audio_reproducer = AudioReproducer(sample_rate=48000, channels=1, chunk_size=960)
        self.audio_reproducer_started = False
        self.audio_reproducer_enabled = False  # Manual control flag
        self.audio_reproducer_lock = threading.Lock()
        
        logger.info("AudioReproducer instance created (manual control mode)")

        self.preallocate_latest_images()

        for i, topic in enumerate(camera_topic):
            print(f"Subscribing to {topic}")
            sub = self.create_subscription(Image, topic, lambda msg, idx=i: self.image_callback(msg, idx), 10)
            self.images_subscribers.append(sub)

        # Add subscriber for WebRTC commands
        self.webrtc_command_subscriber = self.create_subscription(
            String, 
            "/web_rtc_commands", 
            self.webrtc_command_callback, 
            10
        )

        logger.info("Intermediate Node initialized in %s mode", self.mode)

    def receive_client_audio(self, audio_data_float32):
        """
        Receive float32 audio from WebRTC client and feed directly to AudioReproducer.
        Only plays if client audio playback is enabled.
        
        Args:
            audio_data_float32: numpy array of float32 samples in range [-1.0, 1.0]
        """
        with self.audio_reproducer_lock:
            # Only process if client audio playback is enabled
            if not self.audio_reproducer_enabled:
                return  # Discard audio silently
            
            # Start reproducer on first audio data (if enabled)
            if not self.audio_reproducer_started:
                logger.info("Starting AudioReproducer on first client audio (playback enabled)")
                self.audio_reproducer.start()
                self.audio_reproducer_started = True
        
        # Feed audio directly to reproducer
        self.audio_reproducer.feed_audio(audio_data_float32)
    
    def set_main_loop(self, loop):
        """Set reference to main asyncio event loop."""
        self.main_loop = loop

    def image_callback(self, msg, index):
        """
        Callback function to process the incoming image messages.
        """
        
        current_time = time.time()
        if current_time - self.last_time >= 1.0 / self.fps:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self.mode == "manual":
                resized_image = cv2.resize(cv_image, self.manual_resolution)
            else:
                resized_image = self.resize_image(cv_image)

            # Instead of replacing reference, copy data into preallocated buffer:
            if self.latest_images[index].shape == resized_image.shape:
                np.copyto(self.latest_images[index], resized_image)
            else:
                # Fallback (should not happen if you keep resolution consistent)
                self.latest_images[index] = resized_image

            #self.latest_images[index] = resized_image
            self.last_time = current_time

    def webrtc_command_callback(self, msg):
        """
        Callback function to process WebRTC commands.
        
        Commands:
          - toggle_server_audio: Enable/disable server microphone capture (send to client)
          - toggle_client_audio: Enable/disable client audio playback (local speakers)
        """
        command = msg.data
        logger.info(f"Received WebRTC command: {command}")
    
        if command == "toggle_server_audio":
            # Toggle server microphone (captures and sends to client)
            logger.info("Processing toggle_server_audio command")
            if len(self.audio_tracks) > 0:
                # Check if any track is enabled
                any_enabled = any(track.audio_enabled for track in self.audio_tracks)
                if any_enabled:
                    logger.info("Disabling server audio capture")
                    self._direct_stop_audio()
                else:
                    logger.info("Enabling server audio capture")
                    self._direct_start_audio()
            else:
                logger.warning("No audio tracks available for server audio")
        
        elif command == "toggle_client_audio":
            # Toggle client audio playback (plays client audio locally)
            logger.info("Processing toggle_client_audio command")
            with self.audio_reproducer_lock:
                if self.audio_reproducer_enabled:
                    logger.info("Disabling client audio playback")
                    self.audio_reproducer_enabled = False
                    if self.audio_reproducer_started:
                        self.audio_reproducer.stop()
                        self.audio_reproducer_started = False
                else:
                    logger.info("Enabling client audio playback")
                    self.audio_reproducer_enabled = True
                    # Will start on first audio data
        
        else:
            logger.warning(f"Unknown WebRTC command: {command}")
    
    def _direct_start_audio(self):
        """Start audio capture directly (thread-safe)."""
        logger.info(f"_direct_start_audio called. Audio tracks available: {len(self.audio_tracks)}")
        if len(self.audio_tracks) == 0:
            logger.warning("No audio tracks registered! Cannot start audio capture.")
            return
        for track in self.audio_tracks:
            logger.info(f"Starting audio capture for track: {track}")
            # Call the track method directly - it will handle asyncio internally
            track.enable_audio_capture()
        logger.info("_direct_start_audio completed")

    def _direct_stop_audio(self):
        """Stop audio capture directly (thread-safe)."""
        logger.info(f"_direct_stop_audio called. Audio tracks available: {len(self.audio_tracks)}")
        for track in self.audio_tracks:
            logger.info(f"Stopping audio capture for track: {track}")
            track.disable_audio_capture()

    def add_peer(self, peer_id):
        """Add a new peer connection."""
        self.active_peers.add(peer_id)
        logger.info(f"Peer {peer_id} connected. Active peers: {len(self.active_peers)}")

    def remove_peer(self, peer_id):
        """Remove a peer connection."""
        self.active_peers.discard(peer_id)
        logger.info(f"Peer {peer_id} disconnected. Active peers: {len(self.active_peers)}")
        
        # Stop audio if no active peers
        if len(self.active_peers) == 0:
            self.stop_audio_capture()

    def register_audio_track(self, audio_track):
        """Register an audio track."""
        self.audio_tracks.add(audio_track)
        logger.info(f"Audio track registered. Total tracks: {len(self.audio_tracks)}")

    def unregister_audio_track(self, audio_track):
        """Unregister an audio track."""
        self.audio_tracks.discard(audio_track)
        logger.info(f"Audio track unregistered. Total tracks: {len(self.audio_tracks)}")
        
        # Stop audio capture if no tracks need it
        if len(self.audio_tracks) == 0:
            self.stop_audio_capture()

    def start_audio_capture(self):
        """Start audio capture for all registered tracks."""
        for track in self.audio_tracks:
            track.start_audio_capture()

    def stop_audio_capture(self):
        """Stop audio capture for all registered tracks."""
        for track in self.audio_tracks:
            track.stop_audio_capture()

    def update_bandwidth(self, msg):
        """
        Updates the available bandwidth based on message data.
        """
        self.bandwidth = msg.data
        self.adjust_fps_and_resolution()
    
    def preallocate_latest_images(self):
        width, height = self.resolution
        # Shape is (height, width, 3) for BGR images
        shape = (height, width, 3)
        for i in range(self.camera_count):
            # Initialize with zeros, dtype uint8 for images
            self.latest_images[i] = np.zeros(shape, dtype=np.uint8)

    def adjust_fps_and_resolution(self):
        """
        Adjusts FPS and resolution based on the current round-trip time (RTT).
        """
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
        """
        Resize the image based on the current bandwidth.
        """
        if self.rtt is not None:
            if self.rtt is not self.last_rtt:
                self.adjust_fps_and_resolution()
                self.last_rtt = self.rtt
            return cv2.resize(image, self.resolution)
        return image

    def get_latest_image(self, index):
        """
        Returns the latest processed image or a placeholder if none available.
        """
        return self.latest_images[index] if self.latest_images[index] is not None else self.placeholder_image

class ImageVideoTrack(MediaStreamTrack):
    """
    MediaStreamTrack for video, streaming images from the ROS node.
    """
    kind = "video"

    def __init__(self, intermediate_node, index):
        super().__init__()
        self.index = index
        self.start_time = time.time()
        self.frames = 0
        self.framerate = 30
        self.intermediate_node = intermediate_node

    async def next_timestamp(self):
        """
        Calculates the timestamp for the next frame.
        """
        self.frames += 1
        next_time = self.start_time + (self.frames / self.framerate)
        await asyncio.sleep(max(0, next_time - time.time()))
        return int((next_time - self.start_time) * 1000)

    async def recv(self):
        """
        Receives the next video frame to be sent to the peer.
        """
        frame = await self.get_frame()
        image_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        image_frame.pts = await self.next_timestamp()
        image_frame.time_base = Fraction(1, 1000)
        return image_frame

    async def get_frame(self):
        """
        Retrieves the latest image frame from the intermediate node.
        """
        latest_frame = self.intermediate_node.get_latest_image(self.index)
        await asyncio.sleep(1.0 / self.intermediate_node.fps)
        return latest_frame

class AudioStreamTrack(MediaStreamTrack):
    """
    MediaStreamTrack for audio, capturing and streaming audio from microphone using ALSA.
    """
    kind = "audio"

    def __init__(self, main_loop_ref=None):
        super().__init__()
        self.sample_rate = 48000
        self.channels = 1
        self.samples_per_frame = 960  # 20ms at 48kHz
        self.start_time = time.time()
        self.frame_count = 0
        self.audio_enabled = False
        
        # Store reference to main event loop
        self._main_loop_ref = main_loop_ref
        
        # Buffer for smoother audio - 50 frames = 1 second buffer
        self.audio_queue = asyncio.Queue(maxsize=50)
        self.capture_process = None
        self.capture_task = None
        self.producer_task = None
        
        # Flag-based approach for cross-thread communication
        self._start_audio_requested = False
        self._stop_audio_requested = False
        
        self.frame_duration = 1000 / 50  # 20ms in milliseconds
        self.next_frame_time = None
        
        # Audio buffer for smoother playback with minimum buffer size
        self.audio_buffer = []
        self.buffer_target_size = 8  # Target 8 frames ahead for smooth playback (160ms)
        self.buffer_minimum_size = 3  # NEVER go below 3 frames (60ms) - prevents micro-gaps
        self.buffer_warmup_size = 12  # Start playing only after 12 frames (240ms) for initial stability
        
    def enable_audio_capture(self):
        """Enable audio capture (thread-safe method callable from ROS thread)."""
        logger.info("enable_audio_capture called - using flag-based approach")
        self._start_audio_requested = True
        self._stop_audio_requested = False
        logger.info("Audio start flag set - will be processed in next recv() call")
    
    def disable_audio_capture(self):
        """Disable audio capture (thread-safe method callable from ROS thread)."""
        logger.info("disable_audio_capture called - using flag-based approach")
        self._stop_audio_requested = True
        self._start_audio_requested = False
        logger.info("Audio stop flag set - will be processed in next recv() call")
    
    async def _start_audio_producer(self):
        """Start the async audio producer task."""
        logger.info("_start_audio_producer: Starting audio producer")
        try:
            # Check if audio devices are available
            logger.info("_start_audio_producer: Checking audio devices with 'arecord -l'")
            result = await asyncio.create_subprocess_exec(
                'arecord', '-l',
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            await result.wait()
            
            if result.returncode != 0:
                logger.error("No audio input devices available")
                self.audio_enabled = False
                return
            
            logger.info("_start_audio_producer: Audio devices found, starting ALSA subprocess")
            
            # Start audio capture subprocess with  ALSA BUFFER
            cmd = [
                'arecord',
                '-f', 'FLOAT_LE',
                '-c', str(self.channels),
                '-r', str(self.sample_rate),
                '-t', 'raw',
                '--buffer-size', str(self.samples_per_frame * 4 * 10),  # 10x larger buffer (38400 bytes)
                '--period-size', str(self.samples_per_frame * 2),        # Larger periods for stability
                '-'
            ]
            
            logger.info(f"_start_audio_producer: Executing command: {' '.join(cmd)}")
            
            self.capture_process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            
            logger.info(f"_start_audio_producer: ALSA subprocess started with PID: {self.capture_process.pid}")
            logger.info(f"Async ALSA capture started: {self.sample_rate}Hz, {self.channels} channels")
            
            # Start the producer loop
            logger.info("_start_audio_producer: Starting producer loop")
            await self._async_producer_loop()
            
        except Exception as e:
            logger.error(f"Failed to start async audio producer: {e}")
            import traceback
            logger.error(f"Traceback: {traceback.format_exc()}")
            self.audio_enabled = False
    
    async def _async_producer_loop(self):
        """Async producer loop that reads from ALSA and feeds the queue."""
        bytes_per_sample = 4  # float32
        bytes_per_frame = self.samples_per_frame * self.channels * bytes_per_sample
        
        logger.info("Starting async audio producer loop")
        logger.info(f"Reading {bytes_per_frame} bytes per frame ({self.samples_per_frame} samples)")
        logger.info("Audio enabled - transmitting all microphone input")
        
        frame_counter = 0
        
        while self.audio_enabled and self.capture_process:
            try:
                # Async read from subprocess
                audio_data = await self.capture_process.stdout.read(bytes_per_frame)
                
                if not audio_data or len(audio_data) == 0:
                    logger.warning("No audio data from ALSA subprocess")
                    await asyncio.sleep(0.001)  # Small delay before retry
                    continue
                
                # Handle partial reads
                while len(audio_data) < bytes_per_frame and self.audio_enabled:
                    remaining = bytes_per_frame - len(audio_data)
                    additional_data = await self.capture_process.stdout.read(remaining)
                    if additional_data:
                        audio_data += additional_data
                    else:
                        # Pad with zeros if no more data
                        audio_data += b'\x00' * remaining
                        break
                
                frame_counter += 1
                
                # Convert to numpy array
                samples = np.frombuffer(audio_data[:bytes_per_frame], dtype=np.float32)
                
                # Ensure correct sample count
                expected_samples = self.samples_per_frame * self.channels
                if len(samples) != expected_samples:
                    if len(samples) > expected_samples:
                        samples = samples[:expected_samples]
                    else:
                        samples = np.pad(samples, (0, expected_samples - len(samples)))
                
                # Reshape for channels
                if self.channels == 1:
                    samples = samples.reshape(-1, 1)
                else:
                    samples = samples.reshape(-1, self.channels)
                
                # Simple processing - no VAD, transmit everything
                processed_samples = samples
                
                # Light processing to prevent clipping
                processed_samples = processed_samples - np.mean(processed_samples)
                processed_samples = np.clip(processed_samples, -0.95, 0.95)
                
                # Log audio level occasionally for monitoring
                if frame_counter % 100 == 0:  # Log every 2 seconds
                    audio_level = np.sqrt(np.mean(samples ** 2))  # RMS
                    peak_level = np.max(np.abs(samples))  # Peak
                    logger.info(f"Audio transmission - RMS: {audio_level:.4f}, Peak: {peak_level:.4f}")
                
                # Add to asyncio queue (with timeout to prevent blocking)
                try:
                    await asyncio.wait_for(
                        self.audio_queue.put(processed_samples), 
                        timeout=0.005  # 5ms timeout
                    )
                except asyncio.TimeoutError:
                    # Queue is full, drop oldest frames
                    dropped = 0
                    while not self.audio_queue.empty() and dropped < 3:
                        try:
                            self.audio_queue.get_nowait()
                            dropped += 1
                        except asyncio.QueueEmpty:
                            break
                    
                    # Try to add again
                    try:
                        self.audio_queue.put_nowait(processed_samples)
                    except asyncio.QueueFull:
                        logger.warning("Audio queue still full after cleanup")
                
                # Small delay to prevent busy loop
                await asyncio.sleep(0.001)
                        
            except Exception as e:
                if self.audio_enabled:
                    logger.error(f"Async producer error: {e}")
                await asyncio.sleep(0.01)  # Delay on error
        
        logger.info("Async audio producer loop ended")

    async def _get_audio_frame(self):
        """Get audio frame from asyncio queue """
        if not self.audio_enabled:
            return np.zeros((self.samples_per_frame, self.channels), dtype=np.float32)
        
        # Pre-fill buffer for smoother playback
        while len(self.audio_buffer) < self.buffer_target_size and not self.audio_queue.empty():
            try:
                audio_data = self.audio_queue.get_nowait()
                self.audio_buffer.append(audio_data)
            except asyncio.QueueEmpty:
                break
        
        # Wait for warmup buffer on first start for initial stability
        initial_warmup_needed = len(self.audio_buffer) == 0 and self.frame_count < 10
        if initial_warmup_needed:
            logger.info("Audio warmup: Building initial buffer...")
            warmup_count = 0
            while len(self.audio_buffer) < self.buffer_warmup_size and warmup_count < 20:
                try:
                    audio_data = await asyncio.wait_for(self.audio_queue.get(), timeout=0.020)
                    self.audio_buffer.append(audio_data)
                    warmup_count += 1
                except asyncio.TimeoutError:
                    break
            logger.info(f"Audio warmup completed: {len(self.audio_buffer)} frames buffered")
        
        #NEVER drain buffer below minimum - prevents micro-gaps
        if len(self.audio_buffer) > self.buffer_minimum_size:
            audio_data = self.audio_buffer.pop(0)
            
            # Ensure correct frame size
            if len(audio_data) >= self.samples_per_frame:
                return audio_data[:self.samples_per_frame].astype(np.float32)
            else:
                # Pad if needed
                padding = np.zeros((self.samples_per_frame - len(audio_data), self.channels), dtype=np.float32)
                return np.vstack([audio_data, padding]).astype(np.float32)
        
        # IMPROVED: Only if buffer is below minimum, try to get from queue with short timeout
        elif len(self.audio_buffer) > 0:
            # Use existing buffer but try to refill immediately
            try:
                audio_data = await asyncio.wait_for(self.audio_queue.get(), timeout=0.005)
                self.audio_buffer.append(audio_data)
                # Return the oldest frame
                audio_data = self.audio_buffer.pop(0)
                
                if len(audio_data) >= self.samples_per_frame:
                    return audio_data[:self.samples_per_frame].astype(np.float32)
                else:
                    padding = np.zeros((self.samples_per_frame - len(audio_data), self.channels), dtype=np.float32)
                    return np.vstack([audio_data, padding]).astype(np.float32)
                    
            except asyncio.TimeoutError:
                # Buffer is low and no new data - return what we have
                if self.audio_buffer:
                    audio_data = self.audio_buffer.pop(0)
                    if len(audio_data) >= self.samples_per_frame:
                        return audio_data[:self.samples_per_frame].astype(np.float32)
                    else:
                        padding = np.zeros((self.samples_per_frame - len(audio_data), self.channels), dtype=np.float32)
                        return np.vstack([audio_data, padding]).astype(np.float32)
                else:
                    return np.zeros((self.samples_per_frame, self.channels), dtype=np.float32)
        
        # Last resort - buffer is completely empty
        try:
            audio_data = await asyncio.wait_for(self.audio_queue.get(), timeout=0.010)
            
            if len(audio_data) >= self.samples_per_frame:
                return audio_data[:self.samples_per_frame].astype(np.float32)
            else:
                padding = np.zeros((self.samples_per_frame - len(audio_data), self.channels), dtype=np.float32)
                return np.vstack([audio_data, padding]).astype(np.float32)
                
        except asyncio.TimeoutError:
            return np.zeros((self.samples_per_frame, self.channels), dtype=np.float32)
        except Exception as e:
            logger.error(f"Error getting audio frame: {e}")
            return np.zeros((self.samples_per_frame, self.channels), dtype=np.float32)

    async def _stop_audio_internal(self):
        """Internal method to stop audio capture (async context)."""
        logger.info("_stop_audio_internal: Stopping audio capture")
        self.audio_enabled = False
        
        # Cancel producer task
        if self.producer_task:
            logger.info("_stop_audio_internal: Cancelling producer task")
            self.producer_task.cancel()
            try:
                await self.producer_task
            except asyncio.CancelledError:
                logger.info("_stop_audio_internal: Producer task cancelled successfully")
            except Exception as e:
                logger.error(f"_stop_audio_internal: Error cancelling producer task: {e}")
            self.producer_task = None
        
        # Stop ALSA process
        if self.capture_process:
            logger.info("_stop_audio_internal: Terminating ALSA process")
            try:
                self.capture_process.terminate()
                await asyncio.wait_for(self.capture_process.wait(), timeout=2.0)
                logger.info("_stop_audio_internal: ALSA process terminated successfully")
            except asyncio.TimeoutError:
                logger.warning("_stop_audio_internal: ALSA process didn't terminate, killing it")
                self.capture_process.kill()
            except Exception as e:
                logger.error(f"_stop_audio_internal: Error terminating ALSA process: {e}")
            self.capture_process = None
        
        # Clear the queue
        cleared_items = 0
        while not self.audio_queue.empty():
            try:
                self.audio_queue.get_nowait()
                cleared_items += 1
            except:
                break
        
        if cleared_items > 0:
            logger.info(f"_stop_audio_internal: Cleared {cleared_items} items from audio queue")
        
        logger.info("_stop_audio_internal: Audio capture stopped")

    async def recv(self):
        """
        Receives the next audio frame with better timing control.
        """
        # Check if audio start was requested (flag-based approach)
        if self._start_audio_requested and not self.audio_enabled:
            logger.info("recv(): Processing audio start request")
            self._start_audio_requested = False
            self.audio_enabled = True
            
            # Initialize timing
            self.next_frame_time = time.time()
            
            # Start the audio producer task
            try:
                self.producer_task = asyncio.create_task(self._start_audio_producer())
                logger.info("recv(): Audio producer task created successfully")
            except Exception as e:
                logger.error(f"recv(): Failed to create audio producer task: {e}")
                self.audio_enabled = False
        
        # Check if audio stop was requested
        if self._stop_audio_requested and self.audio_enabled:
            logger.info("recv(): Processing audio stop request")
            self._stop_audio_requested = False
            await self._stop_audio_internal()
        
        # Precise timing control for consistent 20ms intervals
        if self.audio_enabled and self.next_frame_time is not None:
            current_time = time.time()
            if current_time < self.next_frame_time:
                sleep_time = self.next_frame_time - current_time
                if sleep_time > 0 and sleep_time < 0.05:  # Max 50ms sleep
                    await asyncio.sleep(sleep_time)
            
            # Schedule next frame time
            self.next_frame_time = current_time + 0.02  # 20ms = 0.02 seconds
        
        # Get audio samples (either from microphone or silence)
        samples = await self._get_audio_frame()
        
        # Ensure correct data type and shape
        samples = samples.astype(np.float32)
        
        # Convert to int16 format for Opus encoder
        # Apply gentle gain and clipping
        samples = np.clip(samples * 0.8, -1.0, 1.0)  # Reduce gain to prevent clipping
        samples_int16 = (samples * 32767).astype(np.int16)
        
        # For mono: transpose to shape (1, samples)
        if self.channels == 1:
            samples_int16 = samples_int16.T
        
        # Create AudioFrame with s16 format
        frame = AudioFrame.from_ndarray(samples_int16, format='s16', layout='mono')
        frame.sample_rate = self.sample_rate
        
        # timestamp calculation
        self.frame_count += 1
        pts = int((self.frame_count * self.samples_per_frame * 1000) / self.sample_rate)
        frame.pts = pts
        frame.time_base = Fraction(1, 1000)
        
        return frame
    
# Web handler functions
async def index(request):
    """
    Serves the index.html page.
    """
    # Updated path to point to the GUI directory in src
    content = open(os.path.join(ROOT, "../GUI/index.html"), "r").read()
    return web.Response(content_type="text/html", text=content)

async def offer(request):
    """
    Handles the WebRTC offer from the client.
    """
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    client_resolutions = params["video_resolution"]

    if intermediate_node.mode == "manual":
        shape = client_resolutions.split('x')
        new_resolution = (int(shape[0]), int(shape[1]))
        intermediate_node.manual_resolution = new_resolution

    pc = RTCPeerConnection()
    pc_id = "PeerConnection(%s)" % uuid.uuid4()
    pcs.add(pc)

    # Register this peer with the intermediate node
    intermediate_node.add_peer(pc_id)

    def log_info(msg, *args):
        """
        Utility function for logging information.
        """
        logger.info(f"{pc_id} {msg}")

    #log_info("Received WebRTC offer")

    # Add video tracks for available cameras
    for index in range(intermediate_node.camera_count):
        if intermediate_node.latest_images[index] is not None:
            log_info("Adding video track for camera " + str(index))
            image_track = ImageVideoTrack(intermediate_node, index)
            pc.addTrack(image_track)
        else:
            log_info("No image subscriber for camera %d", index)

    # Add bidirectional audio track (server to client + client to server)
    log_info("Adding bidirectional audio track")
    audio_track = AudioStreamTrack(main_loop_ref=intermediate_node.main_loop)
    intermediate_node.register_audio_track(audio_track)
    pc.addTrack(audio_track)

    @pc.on("track")
    def on_track(track):
        """Handle incoming audio track from client"""
        log_info(f"Received track: {track.kind}")
        if track.kind == "audio":
            log_info("Setting up client audio reception")
            # Create a task to handle incoming audio frames
            asyncio.create_task(handle_client_audio_track(track))

    async def handle_client_audio_track(track):
        """Process incoming audio frames from client (resample to float mono 48kHz)."""
        log_info("Starting client audio track handler (with resampler)")
        frame_count = 0

        # Initialize resampler to force float mono 48kHz
        try:
            resampler = av.AudioResampler(format='flt', layout='mono', rate=48000)
            log_info("AudioResampler initialized: format=flt, layout=mono, rate=48000")
        except Exception as e:
            log_info(f"Failed to create AudioResampler: {e}")
            resampler = None

        try:
            while True:
                frame = await track.recv()
                if not frame:
                    break

                frame_count += 1

                # Resample/convert frame to desired format
                try:
                    if resampler is not None:
                        out = resampler.resample(frame)
                    else:
                        out = frame
                except Exception as e:
                    log_info(f"Resampling error: {e}")
                    out = frame

                frames_to_process = out if isinstance(out, (list, tuple)) else [out]

                for res_frame in frames_to_process:
                    # Convert resampled frame to numpy array
                    try:
                        audio_data = res_frame.to_ndarray()
                    except Exception as e:
                        log_info(f"to_ndarray error: {e}")
                        continue

                    # Flatten and ensure float32
                    if isinstance(audio_data, np.ndarray):
                        if audio_data.ndim > 1:
                            audio_data = audio_data.flatten()
                        if audio_data.dtype != np.float32:
                            audio_data = audio_data.astype(np.float32)
                    else:
                        audio_data = np.array(audio_data, dtype=np.float32)

                    # Pass the cleaned, resampled float32 audio to the intermediate node
                    intermediate_node.receive_client_audio(audio_data)

                    # # Log stats periodically
                    # if frame_count % 100 == 0:
                    #     try:
                    #         rms = np.sqrt(np.mean(audio_data ** 2))
                    #         peak = np.max(np.abs(audio_data))
                    #         log_info(f"Client audio stats [frame {frame_count}]: RMS={rms:.4f}, Peak={peak:.4f}")
                    #     except Exception:
                    #         pass

        except Exception as e:
            log_info(f"Client audio track ended or error: {e}")
            import traceback
            log_info(f"Traceback: {traceback.format_exc()}")
        finally:
            log_info(f"Client audio track handler finished (processed {frame_count} frames)")

    @pc.on("datachannel")
    def on_datachannel(channel):
        @channel.on("message")
        def on_message(message):
            """
            Processes incoming messages on the data channel.
            """
            if isinstance(message, str) and message.startswith("ping"):
                #log_info("Received ping message", message)
                channel.send("pong" + message[4:])
            if isinstance(message, str) and message.startswith("latency"):
                intermediate_node.rtt = int(message[7:])
                #log_info("Updated RTT to %d", intermediate_node.rtt)

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        """
        Monitors the ICE connection state changes.
        """
        log_info("ICE connection state is %s", pc.iceConnectionState)
        if pc.iceConnectionState == "failed":
            await pc.close()
            pcs.discard(pc)
            # Clean up peer and audio track
            intermediate_node.remove_peer(pc_id)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        """
        Monitors the connection state changes.
        """
        log_info("Connection state is %s", pc.connectionState)
        if pc.connectionState in ["disconnected", "failed", "closed"]:
            # Clean up peer and audio track
            intermediate_node.remove_peer(pc_id)

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )

async def on_shutdown(app):
    """
    Handles the shutdown of the web application, closing all peer connections.
    """
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


def main(args=None):

    parser = argparse.ArgumentParser(
        description="WebRTC audio / video / data-channels demo"
    )
    parser.add_argument("--cert-file", help="SSL certificate file (for HTTPS)")
    parser.add_argument("--key-file", help="SSL key file (for HTTPS)")
    parser.add_argument(
        "--host", default="0.0.0.0", help="Host for HTTP server (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--port", type=int, default=8081, help="Port for HTTP server (default: 8080)"
    )
    parser.add_argument("--verbose", "-v", action="count")
    parser.add_argument("--write-audio", help="Write received audio to a file")
    parser.add_argument("--mode", choices=["manual", "auto"], default="auto", help="Set mode to 'manual' or 'auto' (default: 'auto')")

    # args = parser.parse_args()
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

    # App initialization and ROS node creation
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    gui_path = os.path.join(ROOT, "../GUI")
    app.router.add_static("/static/", gui_path)
    app.router.add_get("/", index)  # Serve the main HTML page
    app.router.add_post("/offer", offer)  # Handle WebRTC offers

    # -  - - - -- -- - - -  - - - --  -----------------

    rclpy.init()
    global intermediate_node
    intermediate_node = Intermediate(mode=args.mode)

    # Start the ROS node in a separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(intermediate_node), daemon=True)
    ros_thread.start()

    # Start the web application
    loop = asyncio.get_event_loop()
    intermediate_node.set_main_loop(loop)
    web.run_app(
        app, access_log=None, host=args.host, port=args.port, ssl_context=ssl_context
    )

    ros_thread.join()

if __name__ == "__main__":
    main()