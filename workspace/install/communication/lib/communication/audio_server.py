#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rclpy.parameter import Parameter

from std_msgs.msg import Bool

import threading

import pyaudio
import wave
import socket
import time


class AudioServer(Node): 

    def __init__(self):
        super().__init__('audio_server')
        self.declare_parameter('frequency', 10)
        self.frequency = self.get_parameter('frequency').value
        self.timer = 1.0 / self.frequency
        self.timer_input = self.create_timer(self.timer, self.input_audio)
        self.timer_output = self.create_timer(self.timer, self.output_audio)

        self.subscription = self.create_subscription(
            Bool,
            'audio_status',
            self.status_audio,
            10)
        self.subscription  # prevent unused variable warning

        # Configuration of connection
        self.HOST = '127.0.0.1'
        self.PORT = 8889
        self.client_conected =  False
        self.CHUNK_CONNECTION = 4096

        # Configuration of audio
        self.CHUNK = 4096  # Number of audio samples per frame
        self.FORMAT = pyaudio.paInt16  # Audio format
        self.CHANNELS_OUT = 2  # Number of audio channels
        self.CHANNELS_IN = 2  # Number of audio channels
        self.RATE_OUT = 48000  # Sampling rate (samples per second)
        self.RATE_IN = 48000 # Sampling rate (samples per second)
        self.alive = False

        self.audio = pyaudio.PyAudio()

        # Open the audio stream for input (microphone)
        self.stream_input = self.audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS_IN,
            rate=self.RATE_IN,
            input=True,
            frames_per_buffer=self.CHUNK,
            input_device_index=2
        )

        # Open the audio stream for output (speaker)
        self.stream_output = self.audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS_OUT,
            rate=self.RATE_OUT,
            output=True,
            frames_per_buffer=self.CHUNK,   
            output_device_index=3
        )

        """
        self.stream_device = self.audio.open(
                format=self.FORMAT,
                channels=2,
                rate=self.RATE_OUT,
                output=True,
                input=True,
                frames_per_buffer=self.CHUNK,
                input_device_index=2,
                output_device_index=2
        )
        """

        self.output_filename = 'audio-recording.wav'

        # Create a TCP socket
        self.server_socket = None
        self.client_conected =  False
        self.client_socket = None

        self.start()

    def status_audio(self, msg):
        self.alive = msg.data
        if self.alive:
            self.set_parameters([Parameter('frequency', Parameter.Type.DOUBLE, 10.0)])
            self.frequency = 10.0
        else:
            self.set_parameters([Parameter('frequency', Parameter.Type.DOUBLE, 0.1)])
            self.frequency = 0.1

        # Cancel existing timers
        if self.timer_input:
            self.timer_input.cancel()
        if self.timer_output:
            self.timer_output.cancel()
        period = 1.0 / self.frequency
        self.timer_input  = self.create_timer(period, self.input_audio)
        self.timer_output = self.create_timer(period, self.output_audio)

    def input_audio(self):
        if self.client_conected and self.alive:
            try:
                self.get_logger().info("sos 1")
                wav_file_wb = wave.open(self.output_filename, "wb")
                wav_file_wb.setnchannels(2)
                wav_file_wb.setsampwidth(2)
                wav_file_wb.setframerate(48000)
                # recording audio and saved
                input_audio = self.stream_input.read(5120, exception_on_overflow=False)
                wav_file_wb.writeframes(input_audio)
                # encode and send audio
                wav_file_rb = wave.open(self.output_filename, "rb")
                data = wav_file_rb.readframes(self.CHUNK)

                if self.client_socket:
                    self.client_socket.sendall(data)

            except Exception as e:
                    self.get_logger().error(f"Error in play_audio [INPUT]: {e}")    
                    time.sleep(5) 

    def output_audio(self):
        if self.client_conected and self.alive:
            try:
                self.get_logger().info("sos 2")
                # Receive audio data from the other device
                data = self.client_socket.recv(self.CHUNK_CONNECTION)
                # Play the received audio data
                self.stream_output.write(data, self.CHUNK)

            except Exception as e:
                self.get_logger().error(f"Error in play_audio [OUTPUT]: {e}")
                time.sleep(5)


    def client_connect(self):  
        while rclpy.ok():      
            if not self.client_conected:
                try:
                    self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.server_socket.bind((self.HOST, self.PORT))
                    self.get_logger().info("Waiting for a client to connect")
                    self.server_socket.listen(1)
                    self.client_socket, self.addr = self.server_socket.accept()
                    self.get_logger().info(f"Conection established with the client {self.addr}") 
                    self.client_conected = True

                except ConnectionRefusedError:
                    self.get_logger().info("Server is down or port blocked.")
                except ConnectionResetError:
                    self.get_logger().info("Server forcibly closed the connection.")
                except TimeoutError:
                    self.get_logger().info("Request timed out.")
                except socket.gaierror:
                    self.get_logger().info("DNS resolution failed (invalid hostname).")
                except OSError as e:  # Catch-all for other socket errors
                    self.get_logger().info(f"Socket error: {e}")
                except Exception as e:
                    self.get_logger().error(f"Socket error: {e}")

            if not self.client_socket:
                self.client_socket.close()
                self.server_socket.close()
                self.client_conected = False
                time.sleep(3)
            time.sleep(1)


    def start(self):
        self.thread = threading.Thread(target=self.client_connect)
        self.thread.daemon = True
        self.thread.start()

    def destroy_node(self):
        if hasattr(self, 'thread'):
            self.thread.join()  # Clean up thread
        if self.client_socket:
            self.client_socket.close()
        self.server_socket.close()
        self.client_connected = False
        self.stream_input.stop_stream()
        self.stream_output.stop_stream()
        self.stream_input.close()
        self.stream_output.close()
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    audio_server = AudioServer()

    try:
        rclpy.spin(audio_server)
    except KeyboardInterrupt:
        pass
    finally:
        audio_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()