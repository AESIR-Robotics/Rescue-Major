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
            # Audio configuration (no cambios en rate, channels, etc.)
            self.RATE = 48000
            self.CHANNELS = 1
            self.FORMAT = pyaudio.paInt16
            self.CHUNK = 480
            
            self.pyaudio_instance = None
            self.input_stream = None
            self.output_stream = None
            self.started = False
            self.output_device_index = None
            
            AudioSystem._initialized = True
            logger.info("AudioSystem initialized")

    def _find_usb_device(self):
        """Busca un dispositivo que contenga 'USB Audio Device' en el nombre."""
        for i in range(self.pyaudio_instance.get_device_count()):
            info = self.pyaudio_instance.get_device_info_by_index(i)
            if "USB Audio Device" in info['name']:
                logger.info(f"USB Audio Device encontrado en índice {i}")
                return i
        return None

    def start(self):
        """Initialize PyAudio and open streams."""
        if self.started:
            logger.warning("AudioSystem already started")
            return
        
        try:
            self.pyaudio_instance = pyaudio.PyAudio()
            
            # Detectar USB Audio Device
            self.output_device_index = self._find_usb_device()
            if self.output_device_index is None:
                logger.warning("No se encontró USB Audio Device, usando dispositivo por defecto")
            
            # Abrir input stream (micrófono)
            self.input_stream = self.pyaudio_instance.open(
                format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.CHUNK,
                stream_callback=None
            )
            
            # Abrir output stream (bocina)
            output_kwargs = dict(
                format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                output=True,
                frames_per_buffer=self.CHUNK,
                stream_callback=None
            )
            if self.output_device_index is not None:
                output_kwargs["output_device_index"] = self.output_device_index
            
            self.output_stream = self.pyaudio_instance.open(**output_kwargs)
            
            # Vaciar buffer inicial
            if self.input_stream.is_active():
                try:
                    for _ in range(5):
                        self.input_stream.read(self.CHUNK, exception_on_overflow=False)
                except Exception:
                    pass
            
            self.started = True
            logger.info(f"AudioSystem started - Rate: {self.RATE}Hz, Channels: {self.CHANNELS}, Chunk: {self.CHUNK}")
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
        """Read audio chunk from microphone (blocking)."""
        if not self.started or not self.input_stream:
            return b'\x00' * (self.CHUNK * 2)
        
        try:
            return self.input_stream.read(self.CHUNK, exception_on_overflow=False)
        except Exception as e:
            logger.error(f"Error reading audio: {e}")
            return b'\x00' * (self.CHUNK * 2)

    def write_audio(self, audio_data):
        """Write audio chunk to speakers (blocking)."""
        if not self.started or not self.output_stream:
            return
        
        try:
            self.output_stream.write(audio_data)
        except Exception as e:
            logger.error(f"Error writing audio: {e}")
