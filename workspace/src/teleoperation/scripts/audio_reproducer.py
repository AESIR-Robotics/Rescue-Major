#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String
import logging
import alsaaudio
import threading
import time
import numpy as np
from collections import deque

class AudioReproducer(Node):
    def __init__(self):
        super().__init__("audio_reproducer")
        self.subscription = self.create_subscription(String, "audio_commands", self.listener_callback, 10)
        self.subscription_audio = self.create_subscription(Int16MultiArray, "client_audio", self.audio_callback, 10)

        logging.info("Audio Reproducer node initialized")
        self.audio_flag = False
        self.audio_enabled = False  
        # Jitter buffer for live audio streaming (~1 second max)
        self.audio_buffer = deque(maxlen=50)
        self.audio_lock = threading.Lock()
        self.last_audio_time = 0
        self.audio_timeout = 0.5
        self.is_receiving_audio = False
        
        # Jitter buffer settings
        self.min_buffer_size = 2  # Min chunks before playback starts (40ms initial latency)
        self.max_buffer_size = 10  # Max chunks (200ms) - discard older if exceeded
        self.target_buffer_size = 3  # Target size (60ms)
        
        # Underrun handling
        self.last_audio_chunk = None
        self.underrun_count = 0
        
        # Thread control
        self.audio_thread = None
        self.thread_lock = threading.Lock()

    def listener_callback(self, msg):
        logging.info(f"Received audio command: {msg.data}")

        if msg.data == "toggle_audio":
            logging.info("Processing toggle_audio command")
            if self.audio_enabled:
                # Audio is currently ON, turn it OFF
                logging.info("Turning audio playback OFF")
                self.audio_enabled = False
                self.audio_flag = False
            else:
                # Audio is currently OFF, turn it ON
                logging.info("Turning audio playback ON")
                self.audio_enabled = True
                with self.thread_lock:
                    if self.audio_thread is None or not self.audio_thread.is_alive():
                        self.audio_flag = True
                        self.audio_thread = threading.Thread(target=self.reproduce, daemon=True)
                        self.audio_thread.start()
                        logging.info("Audio playback started")
                    else:
                        logging.warning("Audio playback already running")
        else:
            logging.warning(f"Unknown audio command: {msg.data}")

    def audio_callback(self, msg):
        """Receive audio from client via client_audio topic"""
        current_time = time.time()
        
        with self.audio_lock:
            if len(msg.data) > 0:
                audio_array = np.array(msg.data, dtype=np.int16)
                
                # Detect real audio content (not just silence)
                has_audio_content = np.any(np.abs(audio_array) > 100)
                
                if has_audio_content:
                    buffer_size = len(self.audio_buffer)
                    
                    # Prevent buffer overflow by discarding old chunks
                    if buffer_size >= self.max_buffer_size:
                        discard_count = buffer_size - self.target_buffer_size
                        for _ in range(discard_count):
                            self.audio_buffer.popleft()
                        logging.warning(f"Buffer overflow - discarded {discard_count} old chunks")
                    
                    self.audio_buffer.append(audio_array)
                    self.last_audio_time = current_time
                    
                    if not self.is_receiving_audio:
                        self.is_receiving_audio = True
                        logging.info("Started receiving client audio for playback")
                else:
                    # Check timeout for silence
                    if self.is_receiving_audio and (current_time - self.last_audio_time) > self.audio_timeout:
                        self.is_receiving_audio = False
                        logging.info("Stopped receiving client audio - timeout")

    def reproduce(self):
        logging.info("Reproducing audio...")
        
        # Configure ALSA playback device
        try:
            device = alsaaudio.PCM(alsaaudio.PCM_PLAYBACK)
            device.setchannels(1)
            device.setrate(48000)
            device.setformat(alsaaudio.PCM_FORMAT_S16_LE)
            device.setperiodsize(960)  # Match sender: 20ms @ 48kHz
            logging.info("Audio playback device configured successfully")
        except Exception as e:
            logging.error(f"Error configuring audio device: {e}")
            self.audio_flag = False
            return
        
        self.audio_flag = True
        playback_started = False
        
        while self.audio_flag:
            try:
                audio_data = None
                buffer_size = 0
                
                with self.audio_lock:
                    buffer_size = len(self.audio_buffer)
                    
                    # Wait for minimum buffer before starting (reduces initial jitter)
                    if not playback_started:
                        if buffer_size >= self.min_buffer_size:
                            playback_started = True
                            logging.info(f"Starting playback with {buffer_size} chunks buffered")
                    
                    if playback_started and buffer_size > 0:
                        audio_data = self.audio_buffer.popleft()
                        self.last_audio_chunk = audio_data
                
                if audio_data is not None:
                    audio_bytes = audio_data.tobytes()
                    device.write(audio_bytes)
                    self.underrun_count = 0
                    
                elif playback_started:
                    # Underrun: no data available but already playing
                    self.underrun_count += 1
                    
                    if self.underrun_count < 5:
                        # Smooth underrun with fade-out or silence
                        if self.last_audio_chunk is not None:
                            faded_chunk = self.last_audio_chunk.copy()
                            fade = np.linspace(1.0, 0.0, len(faded_chunk))
                            faded_chunk = (faded_chunk * fade).astype(np.int16)
                            device.write(faded_chunk.tobytes())
                        else:
                            silence = np.zeros(960, dtype=np.int16)
                            device.write(silence.tobytes())
                    else:
                        # Too many underruns - reset playback
                        logging.warning("Multiple underruns detected - resetting playback")
                        playback_started = False
                        self.underrun_count = 0
                        time.sleep(0.1)
                else:
                    # Waiting for buffer to fill
                    time.sleep(0.005)
                    
            except Exception as e:
                logging.error(f"Error in reproduce loop: {e}")
                time.sleep(0.1)
        
        # Cleanup audio device
        try:
            device.close()
            logging.info("Audio playback device closed")
        except:
            pass
        
        logging.info("Audio reproduction stopped")


def main(args=None):
    rclpy.init(args=args)
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    audio_reproducer = AudioReproducer()
    
    try:
        rclpy.spin(audio_reproducer)
    except KeyboardInterrupt:
        logging.info("Audio reproducer node interrupted")
    finally:
        audio_reproducer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
