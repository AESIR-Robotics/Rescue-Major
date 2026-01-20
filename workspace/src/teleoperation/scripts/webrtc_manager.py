"""
WebRTC Manager Module
Handles PeerConnection setup, transceivers configuration, and WebRTC offer/answer logic.
"""
import asyncio
import json
import logging
import uuid
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiohttp import web
from media_tracks import ImageVideoTrack

logger = logging.getLogger("pc")


class WebRTCManager:
    def __init__(self):
        self.pcs = set()
    
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

        pc = RTCPeerConnection()
        pc_id = "PeerConnection(%s)" % uuid.uuid4()
        self.pcs.add(pc)

        def log_info(msg, *args):
            """Utility function for logging information."""
            logger.info(f"{pc_id} {msg}")

        # Add video tracks for available cameras
        for index in range(intermediate_node.camera_count):
            if intermediate_node.latest_images[index] is not None:
                log_info("Adding video track for camera " + str(index))
                image_track = ImageVideoTrack(intermediate_node, index)
                pc.addTrack(image_track)
            else:
                log_info("No image subscriber for camera %d", index)

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
