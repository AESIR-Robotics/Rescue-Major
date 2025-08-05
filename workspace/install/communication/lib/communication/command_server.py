#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from base_socket import ConnectionBase
import threading
import logging  

logger = logging.getLogger(__name__)  
logging.basicConfig(level=logging.DEBUG, format=':%(message)s')

class Server(ConnectionBase):
    def __init__(self, mode, port):
        super().__init__(mode, port)
        self.gather_info = input
        self.notify = print
        
    def bind_socket_ifneeded(self):
        self.socket.bind((self.host, self.port))

class Publisher(Node):
    def __init__(self):
        logger.info("Startinf publisher node")
        super().__init__("publisher_node_client_info")
        self.publisher_ = self.create_publisher(String, "Distribution_of_client_commands", 10)
        self.server = Server("server", "communication_port")
        self.server.notify = self.publish_from_buffer
        server_thread = threading.Thread(target=self.server.start)

        #The method from the server treiggers the publish method
        server_thread.start()

    def publish_from_buffer(self):
        msg = String()
        msg.data = self.server.buffer
        logger.debug(f'Publishing: {msg.data}')
        self.publisher_.publish(msg)

""""
class Subscriber(Node):
    def __init__(self,socket_class):
        super().init__("Gather info to send ")
        self.sok = socket_class
        self.subscription = self.create_subscription(string.String, "Send information to server", self.listener_callback, 10)

    def listener_callback(self, msg):
        self.sok.manage_received_info(msg.data)
"""

def main (args=None):
    rclpy.init(args=args)
    pub = Publisher()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
