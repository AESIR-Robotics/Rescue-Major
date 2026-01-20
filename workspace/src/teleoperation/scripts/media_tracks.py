"""
Media Tracks Module
Contains custom MediaStreamTrack implementations for WebRTC streaming.
"""
import asyncio
import time
from fractions import Fraction
from aiortc import MediaStreamTrack
from av import VideoFrame


class ImageVideoTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, intermediate_node, index):
        super().__init__()
        self.index = index
        self.start_time = time.time()
        self.frames = 0
        self.framerate = 30
        self.intermediate_node = intermediate_node

    async def next_timestamp(self):
        self.frames += 1
        next_time = self.start_time + (self.frames / self.framerate)
        await asyncio.sleep(max(0, next_time - time.time()))
        return int((next_time - self.start_time) * 1000)

    async def recv(self):
        frame = await self.get_frame()
        image_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        image_frame.pts = await self.next_timestamp()
        image_frame.time_base = Fraction(1, 1000)
        return image_frame

    async def get_frame(self):
        latest_frame = self.intermediate_node.get_latest_image(self.index)
        await asyncio.sleep(1.0 / self.intermediate_node.fps)
        return latest_frame
