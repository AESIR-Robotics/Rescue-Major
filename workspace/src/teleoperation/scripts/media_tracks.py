"""
Media Tracks Module
Contains custom MediaStreamTrack implementations for WebRTC streaming.
"""
import asyncio
import time
import logging
from fractions import Fraction
from aiortc import MediaStreamTrack
from av import VideoFrame, AudioFrame
import numpy as np
from audio_system import AudioSystem

logger = logging.getLogger("media_tracks")


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
        image_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        image_frame = image_frame.reformat(format="yuv420p")
        image_frame.pts = await self.next_timestamp()
        image_frame.time_base = Fraction(1, 1000)
        return image_frame

    async def get_frame(self):
        latest_frame = self.intermediate_node.get_latest_image(self.index)
        await asyncio.sleep(1.0 / self.intermediate_node.fps)
        return latest_frame


class ServerMicTrack(MediaStreamTrack):
    """
    Audio track that sends microphone audio to the WebRTC client.
    Reads from AudioSystem and sends as AudioFrames.
    """
    kind = "audio"

    def __init__(self, audio_system, loop=None):
        super().__init__()
        self.audio_system = audio_system
        self.loop = loop or asyncio.get_event_loop()
        self.samples_count = 0  # Cumulative sample counter for PTS
        self.muted = True  # Soft mute state (starts muted)
        logger.info("ServerMicTrack initialized (muted by default)")
    
    def mute(self):
        self.muted = True
        logger.info("ServerMicTrack muted (soft mute - hardware still active)")
    
    def unmute(self):
        self.muted = False
        
        # Flush any accumulated audio buffers to prevent playback of old audio
        if self.audio_system.input_stream and self.audio_system.input_stream.is_active():
            try:
                # Drain up to 1 second of buffered audio
                available = self.audio_system.input_stream.get_read_available()
                if available > 0:
                    chunks_to_flush = min(available // self.audio_system.CHUNK, 50)  # Max 50 chunks
                    for _ in range(chunks_to_flush):
                        self.audio_system.input_stream.read(self.audio_system.CHUNK, exception_on_overflow=False)
                    logger.info(f"Flushed {chunks_to_flush} audio chunks (~{chunks_to_flush * self.audio_system.CHUNK / self.audio_system.RATE * 1000:.0f}ms)")
            except Exception as e:
                logger.warning(f"Could not flush audio buffer: {e}")
        
        logger.info("ServerMicTrack unmuted")

    async def recv(self):
        """
        Generate and return the next audio frame.
        Reads from hardware to drain buffers and prevent latency.
        Sends silence if muted (enables Opus DTX for bandwidth savings).
        """
        # Read audio in a thread pool to avoid blocking
        audio_data = await self.loop.run_in_executor(
            None,
            self.audio_system.read_audio
        )
        
        # Decide what to send based on mute state
        if self.muted:
            # Discard real data and create silence buffer
            # 960 samples * 2 bytes (int16) = 1920 bytes
            audio_data = bytes(self.audio_system.CHUNK * 2)
        
        # Convert bytes to numpy array (int16)
        audio_array = np.frombuffer(audio_data, dtype=np.int16)
        
        # Reshape to 2D: (channels, samples) for packed format - mono = (1, N)
        audio_array = audio_array.reshape(1, -1)
        
        # Create AudioFrame
        frame = AudioFrame.from_ndarray(
            audio_array,
            format='s16',
            layout='mono'
        )
        frame.sample_rate = self.audio_system.RATE
        
        # Set PTS using cumulative sample count
        frame.pts = self.samples_count
        frame.time_base = Fraction(1, self.audio_system.RATE)
        
        # Increment sample counter (maintains sync even during silence)
        self.samples_count += self.audio_system.CHUNK
        
        return frame
