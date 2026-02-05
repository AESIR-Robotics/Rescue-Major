"""
Audio System Module
Handles PyAudio hardware management for bidirectional audio streaming.
"""
import logging
import pyaudio

logger = logging.getLogger("audio")


class AudioSystem:
    _instance = None
    _initialized = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(AudioSystem, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if not AudioSystem._initialized:
            # Audio configuration
            self.RATE = 48000
            self.CHANNELS = 1
            self.FORMAT = pyaudio.paInt16
            self.CHUNK = 480  # 10ms at 48kHz (reduced from 20ms for lower latency)
            
            self.pyaudio_instance = None
            self.input_stream = None
            self.output_stream = None
            self.started = False
            
            AudioSystem._initialized = True
            logger.info("AudioSystem initialized")

    def start(self):
        """Initialize PyAudio and open streams."""
        if self.started:
            logger.warning("AudioSystem already started")
            return
        
        try:
            self.pyaudio_instance = pyaudio.PyAudio()
            
            # Open input stream (microphone) with minimal latency settings
            self.input_stream = self.pyaudio_instance.open(
                format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.CHUNK,
                stream_callback=None  # Blocking mode for better control
            )
            
            # Open output stream (speakers) with minimal latency
            self.output_stream = self.pyaudio_instance.open(
                format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                output=True,
                frames_per_buffer=self.CHUNK,
                stream_callback=None  # Blocking mode
            )
            
            # Flush any existing buffers
            if self.input_stream.is_active():
                try:
                    # Drain initial buffer to avoid accumulated latency
                    for _ in range(5):
                        self.input_stream.read(self.CHUNK, exception_on_overflow=False)
                except:
                    pass
            
            self.started = True
            logger.info(f"AudioSystem started - Rate: {self.RATE}Hz, Channels: {self.CHANNELS}, Chunk: {self.CHUNK}")
            logger.info("Low-latency mode enabled")
        except Exception as e:
            logger.error(f"Failed to start AudioSystem: {e}")
            self.stop()
            raise

    def stop(self):
        """Close PyAudio streams and terminate."""
        if not self.started:
            return
        
        try:
            if self.input_stream:
                self.input_stream.stop_stream()
                self.input_stream.close()
            if self.output_stream:
                self.output_stream.stop_stream()
                self.output_stream.close()
            if self.pyaudio_instance:
                self.pyaudio_instance.terminate()
            
            self.started = False
            logger.info("AudioSystem stopped")
        except Exception as e:
            logger.error(f"Error stopping AudioSystem: {e}")

    def read_audio(self):
        """
        Read audio chunk from microphone (blocking).
        Returns bytes (960 samples * 2 bytes = 1920 bytes).
        """
        if not self.started or not self.input_stream:
            return b'\x00' * (self.CHUNK * 2)  # Return silence
        
        try:
            return self.input_stream.read(self.CHUNK, exception_on_overflow=False)
        except Exception as e:
            logger.error(f"Error reading audio: {e}")
            return b'\x00' * (self.CHUNK * 2)

    def write_audio(self, audio_data):
        """
        Write audio chunk to speakers (blocking).
        audio_data: bytes (raw PCM data).
        """
        if not self.started or not self.output_stream:
            return
        
        try:
            self.output_stream.write(audio_data)
        except Exception as e:
            logger.error(f"Error writing audio: {e}")
