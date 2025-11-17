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

class AudioReproducer(Node):
    def __init__(self):
        super().__init__("audio_reproducer")
        self.subscription = self.create_subscription(String, "audio_commands", self.listener_callback, 10)
        self.subscription_audio = self.create_subscription(Int16MultiArray, "client_audio", self.audio_callback, 10)

        logging.info("Audio Reproducer node initialized")
        self.audio_flag = False
        
        # Buffer para audio recibido del cliente
        self.audio_buffer = []
        self.audio_lock = threading.Lock()
        self.last_audio_time = 0
        self.audio_timeout = 0.5  # 500ms timeout
        self.is_receiving_audio = False

    def listener_callback(self, msg):
        logging.info(f"Received audio command: {msg.data}")

        if msg.data == "start":
            self.audio_flag = True
            audio_thread = threading.Thread(target=self.reproduce)
            audio_thread.start()
        else:
            self.audio_flag = False

    def audio_callback(self, msg):
        """Callback para recibir audio del cliente desde el tópico client_audio"""
        current_time = time.time()
        
        with self.audio_lock:
            # Verificar si hay datos de audio
            if len(msg.data) > 0:
                # Detectar si hay contenido de audio real (no solo silencio)
                has_audio_content = any(abs(sample) > 100 for sample in msg.data)
                
                if has_audio_content:
                    self.audio_buffer.append(msg.data)
                    self.last_audio_time = current_time
                    
                    if not self.is_receiving_audio:
                        self.is_receiving_audio = True
                        logging.info("Started receiving client audio for playback")
                else:
                    # Solo silencio - verificar timeout
                    if self.is_receiving_audio and (current_time - self.last_audio_time) > self.audio_timeout:
                        self.is_receiving_audio = False
                        logging.info("Stopped receiving client audio - timeout")

    def reproduce(self):
        logging.info("Reproducing audio...")
        
        # Configurar dispositivo de audio ALSA para reproducción
        try:
            device = alsaaudio.PCM(alsaaudio.PCM_PLAYBACK)
            device.setchannels(1)  # Mono
            device.setrate(48000)  # Sample rate
            device.setformat(alsaaudio.PCM_FORMAT_S16_LE)
            device.setperiodsize(1024)
            logging.info("Audio playback device configured successfully")
        except Exception as e:
            logging.error(f"Error configuring audio device: {e}")
            self.audio_flag = False
            return
        
        while self.audio_flag:
            try:
                # Verificar si hay datos de audio en el buffer
                audio_data = None
                with self.audio_lock:
                    if self.audio_buffer:
                        audio_data = self.audio_buffer.pop(0)
                
                if audio_data is not None:
                    # Reproducir los datos de audio
                    try:
                        # Convertir lista a bytes si es necesario
                        if isinstance(audio_data, list):
                            audio_bytes = bytearray()
                            for sample in audio_data:
                                # Asegurar que el sample esté en el rango correcto
                                sample = max(-32768, min(32767, int(sample)))
                                audio_bytes.extend(sample.to_bytes(2, byteorder='little', signed=True))
                        else:
                            # Si ya es numpy array o bytes
                            audio_bytes = audio_data.tobytes() if hasattr(audio_data, 'tobytes') else audio_data
                        
                        # Reproducir audio
                        device.write(bytes(audio_bytes))
                        logging.debug(f"Reproduced audio chunk: {len(audio_data)} samples")
                        
                    except Exception as e:
                        logging.error(f"Error playing audio chunk: {e}")
                else:
                    # No hay audio para reproducir, esperar un poco
                    time.sleep(0.01)  # 10ms
                    
            except Exception as e:
                logging.error(f"Error in reproduce loop: {e}")
                time.sleep(0.1)
        
        # Limpiar dispositivo de audio al salir
        try:
            device.close()
            logging.info("Audio playback device closed")
        except:
            pass
        
        logging.info("Audio reproduction stopped")


def main(args=None):
    rclpy.init(args=args)
    
    # Configure logging
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
