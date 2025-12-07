import logging
import alsaaudio
import threading
import time
import numpy as np
from collections import deque

class AudioReproducer:
    """
    Standalone audio reproducer that accepts float32 audio samples
    and plays them through ALSA with minimal latency.
    """
    def __init__(self, sample_rate=48000, channels=1, chunk_size=960):
        """
        Initialize AudioReproducer
        
        Args:
            sample_rate: Audio sample rate (default 48000 Hz for WebRTC)
            channels: Number of audio channels (default 1 for mono)
            chunk_size: Samples per chunk (default 960 = 20ms @ 48kHz)
        """
        logging.info("AudioReproducer initialized (standalone mode)")
        
        self.sample_rate = sample_rate
        self.channels = channels
        self.chunk_size = chunk_size
        
        self.audio_flag = False
        self.audio_enabled = False  
        
        # Jitter buffer for live audio streaming 
        self.audio_buffer = deque(maxlen=30)  # Max 30 chunks = 600ms
        self.audio_lock = threading.Lock()
        
        # Jitter buffer settings - OPTIMIZED for low latency
        self.min_buffer_size = 1  # Start immediately (20ms latency)
        self.max_buffer_size = 6  # Max 6 chunks (120ms)
        self.target_buffer_size = 2  # Target 2 chunks (40ms)
        
        # Underrun handling
        self.last_audio_chunk = None
        self.underrun_count = 0
        
        # Thread control
        self.audio_thread = None
        self.thread_lock = threading.Lock()
        
        # Stats
        self.frames_received = 0
        self.frames_played = 0
        self.underruns = 0

    def start(self):
        """Start audio playback thread"""
        logging.info("Starting audio playback")
        self.audio_enabled = True
        
        with self.thread_lock:
            if self.audio_thread is None or not self.audio_thread.is_alive():
                self.audio_flag = True
                self.audio_thread = threading.Thread(target=self._playback_loop, daemon=True)
                self.audio_thread.start()
                logging.info("Audio playback thread started")
            else:
                logging.warning("Audio playback already running")
    
    def stop(self):
        """Stop audio playback thread"""
        logging.info("Stopping audio playback")
        self.audio_enabled = False
        self.audio_flag = False
        
        # Wait for thread to finish
        if self.audio_thread and self.audio_thread.is_alive():
            self.audio_thread.join(timeout=2.0)
            logging.info("Audio playback thread stopped")
    
    def feed_audio(self, audio_samples):
        """
        Feed audio samples to the playback buffer.
        Accepts float32 samples in range [-1.0, 1.0] from WebRTC/Opus decoder.
        
        Args:
            audio_samples: numpy array of float32 samples, shape (samples,) or (samples, channels)
        """
        if not isinstance(audio_samples, np.ndarray):
            audio_samples = np.array(audio_samples, dtype=np.float32)
        
        # Ensure float32
        if audio_samples.dtype != np.float32:
            audio_samples = audio_samples.astype(np.float32)
        
        # Flatten if needed
        if audio_samples.ndim > 1:
            audio_samples = audio_samples.flatten()
        
        with self.audio_lock:
            buffer_size = len(self.audio_buffer)
            
            # Prevent buffer overflow by discarding old chunks
            if buffer_size >= self.max_buffer_size:
                discard_count = buffer_size - self.target_buffer_size
                for _ in range(discard_count):
                    self.audio_buffer.popleft()
                logging.debug(f"Buffer overflow - discarded {discard_count} old chunks")
            
            self.audio_buffer.append(audio_samples)
            self.frames_received += 1

    def _playback_loop(self):
        """
        Internal playback loop that reads from buffer and writes to ALSA.
        Converts float32 [-1.0, 1.0] to int16 PCM for ALSA.
        """
        logging.info("Starting audio playback loop...")
        
        # Configure ALSA playback device
        try:
            device = alsaaudio.PCM(alsaaudio.PCM_PLAYBACK)
            device.setchannels(self.channels)
            device.setrate(self.sample_rate)
            device.setformat(alsaaudio.PCM_FORMAT_S16_LE)
            device.setperiodsize(self.chunk_size)
            logging.info(f"ALSA configured: {self.sample_rate}Hz, {self.channels}ch, {self.chunk_size} samples/period")
        except Exception as e:
            logging.error(f"Error configuring ALSA device: {e}")
            self.audio_flag = False
            return
        
        self.audio_flag = True
        playback_started = False
        
        while self.audio_flag:
            try:
                audio_data_float = None
                buffer_size = 0
                
                with self.audio_lock:
                    buffer_size = len(self.audio_buffer)
                    
                    # Wait for minimum buffer before starting (reduces initial jitter)
                    if not playback_started:
                        if buffer_size >= self.min_buffer_size:
                            playback_started = True
                            logging.info(f"Starting playback with {buffer_size} chunks buffered")
                    
                    if playback_started and buffer_size > 0:
                        audio_data_float = self.audio_buffer.popleft()
                        self.last_audio_chunk = audio_data_float
                
                if audio_data_float is not None:
                    # Convert float32 [-1.0, 1.0] to int16 PCM
                    # Clip to prevent overflow, then scale to int16 range
                    audio_clipped = np.clip(audio_data_float, -1.0, 1.0)
                    audio_int16 = (audio_clipped * 32767.0).astype(np.int16)
                    
                    # Write to ALSA
                    audio_bytes = audio_int16.tobytes()
                    device.write(audio_bytes)
                    
                    self.frames_played += 1
                    self.underrun_count = 0
                    
                elif playback_started:
                    # Underrun: no data available but already playing
                    self.underrun_count += 1
                    self.underruns += 1
                    
                    if self.underrun_count < 5:
                        # Smooth underrun with fade-out or silence
                        if self.last_audio_chunk is not None:
                            faded_chunk = self.last_audio_chunk.copy()
                            fade = np.linspace(1.0, 0.0, len(faded_chunk), dtype=np.float32)
                            faded_float = faded_chunk * fade
                            faded_int16 = (np.clip(faded_float, -1.0, 1.0) * 32767.0).astype(np.int16)
                            device.write(faded_int16.tobytes())
                        else:
                            silence = np.zeros(self.chunk_size, dtype=np.int16)
                            device.write(silence.tobytes())
                    else:
                        # Too many underruns - reset playback
                        logging.warning(f"Multiple underruns detected - resetting playback (stats: rx={self.frames_received}, played={self.frames_played}, underruns={self.underruns})")
                        playback_started = False
                        self.underrun_count = 0
                        time.sleep(0.05)
                else:
                    # Waiting for buffer to fill
                    time.sleep(0.002)  # 2ms sleep - very short
                    
            except Exception as e:
                logging.error(f"Error in playback loop: {e}")
                import traceback
                logging.error(traceback.format_exc())
                time.sleep(0.1)
        
        # Cleanup audio device
        try:
            device.close()
            logging.info(f"ALSA device closed (stats: rx={self.frames_received}, played={self.frames_played}, underruns={self.underruns})")
        except:
            pass
        
        logging.info("Audio playback loop stopped")
