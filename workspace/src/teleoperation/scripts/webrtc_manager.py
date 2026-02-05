"""
WebRTC Manager Module
Handles PeerConnection setup, transceivers configuration, and WebRTC offer/answer logic.
"""
import asyncio
import json
import logging
import uuid
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer
from aiohttp import web
from media_tracks import ImageVideoTrack, ServerMicTrack
from audio_system import AudioSystem

logger = logging.getLogger("pc")


class WebRTCManager:
    def __init__(self):
        self.pcs = set()
        self.audio_system = AudioSystem()
        self.audio_initialized = False
        self.mic_track = None  # Reference to active microphone track
    
    def ensure_audio_started(self):
        if not self.audio_initialized:
            try:
                self.audio_system.start()
                self.audio_initialized = True
                logger.info("Audio system initialized successfully")
                return True
            except Exception as e:
                logger.warning(f"Audio system not available: {e}")
                return False
        return self.audio_initialized
    
    def mute_audio(self):
        if self.mic_track:
            self.mic_track.mute()
            logger.info("Microphone muted")
        else:
            logger.warning("Cannot mute - no active microphone track")
    
    def unmute_audio(self):
        if self.mic_track:
            self.mic_track.unmute()
            logger.info("Microphone unmuted")
        else:
            logger.warning("Cannot unmute - no active microphone track")
    
    async def handle_incoming_audio(self, track):
        logger.info("Started handling incoming audio track")
        loop = asyncio.get_event_loop()
        
        try:
            while True:
                # Receive audio frame from client
                frame = await track.recv()
                
                # Convert AudioFrame to bytes (s16 format)
                audio_array = frame.to_ndarray()
                
                # Ensure correct shape and format
                if audio_array.dtype != np.int16:
                    audio_array = audio_array.astype(np.int16)
                
                # Flatten to 1D if needed
                audio_bytes = audio_array.tobytes()
                
                # Write to speakers in thread pool (non-blocking)
                await loop.run_in_executor(
                    None,
                    self.audio_system.write_audio,
                    audio_bytes
                )
        except Exception as e:
            logger.error(f"Error handling incoming audio: {e}")
        finally:
            logger.info("Stopped handling incoming audio track")
    
    async def handle_offer(self, request, intermediate_node):
        """
        Args:
            request: aiohttp request containing the WebRTC offer
            intermediate_node: Reference to the ROS Intermediate node
        """
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        client_resolutions = params["video_resolution"]

        if intermediate_node.mode == "manual":
            shape = client_resolutions.split('x')
            new_resolution = (int(shape[0]), int(shape[1]))
            intermediate_node.manual_resolution = new_resolution

        # Configure ICE servers for NAT traversal
        configuration = RTCConfiguration(
            iceServers=[
                RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
                RTCIceServer(urls=["stun:stun1.l.google.com:19302"]),
                RTCIceServer(urls=["stun:stun2.l.google.com:19302"]),
                RTCIceServer(urls=["stun:stun3.l.google.com:19302"]),
                RTCIceServer(urls=["stun:stun4.l.google.com:19302"]),
            ]
        )
        
        pc = RTCPeerConnection(configuration=configuration)
        pc_id = "PeerConnection(%s)" % uuid.uuid4()
        self.pcs.add(pc)

        def log_info(msg, *args):
            """Utility function for logging information."""
            logger.info(f"{pc_id} {msg}", *args)

        # Initialize audio system if not already started
        audio_available = self.ensure_audio_started()

        # Add video tracks using addTransceiver (same as audio)
        for index in range(intermediate_node.camera_count):
            log_info("Adding video track for camera %d", index)
            image_track = ImageVideoTrack(intermediate_node, index)
            pc.addTransceiver(image_track)

        # Add bidirectional audio track only if audio system is available
        if audio_available:
            try:
                mic_track = ServerMicTrack(self.audio_system)
                pc.addTransceiver(mic_track, direction="sendrecv")
                self.mic_track = mic_track  # Store reference for mute control
                log_info("Added bidirectional audio transceiver")
            except Exception as e:
                logger.warning(f"Could not add audio transceiver: {e}")
        else:
            log_info("Audio track not added - microphone not available")

        @pc.on("track")
        async def on_track(track):
            """Handle incoming tracks from client."""
            log_info(f"Track received: {track.kind}")
            
            if track.kind == "audio":
                # Handle incoming audio from client
                asyncio.ensure_future(self.handle_incoming_audio(track))
            elif track.kind == "video":
                log_info("Received video track (not processed)")

        @pc.on("datachannel")
        def on_datachannel(channel):
            @channel.on("message")
            def on_message(message):
                """Processes incoming messages on the data channel."""
                if isinstance(message, str) and message.startswith("ping"):
                    channel.send("pong" + message[4:])
                if isinstance(message, str) and message.startswith("latency"):
                    intermediate_node.rtt = int(message[7:])

        @pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            """Monitors the ICE connection state changes."""
            log_info("ICE connection state is %s", pc.iceConnectionState)
            if pc.iceConnectionState == "failed":
                await pc.close()
                self.pcs.discard(pc)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            """Monitors the connection state changes."""
            log_info("Connection state is %s", pc.connectionState)
            if pc.connectionState in ["disconnected", "failed", "closed"]:
                pass

        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        # Wait for ICE gathering to complete before sending answer
        # This is critical for non-localhost connections
        await asyncio.sleep(0.1)  # Small delay to allow ICE candidates to start gathering
        
        while pc.iceGatheringState == "gathering":
            await asyncio.sleep(0.05)
        
        log_info("ICE gathering complete, sending answer with state: %s", pc.iceGatheringState)

        return web.Response(
            content_type="application/json",
            text=json.dumps(
                {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
            ),
        )
    
    async def shutdown(self):
        """Closes all peer connections."""
        coros = [pc.close() for pc in self.pcs]
        await asyncio.gather(*coros)
        self.pcs.clear()
        
        # Stop audio system
        if self.audio_initialized:
            self.audio_system.stop()
            self.audio_initialized = False
            logger.info("Audio system stopped")
