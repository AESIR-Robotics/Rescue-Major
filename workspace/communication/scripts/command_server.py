#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
import websockets 
from websockets import serve
import logging
import json 
import asyncio

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG, format=':%(levelname)s:%(message)s') 

class Publisher(Node):
    def __init__(self):
        logger.info("Starting publisher node")
        super().__init__("command_server_pub")
        self.publisher_ = self.create_publisher(String, "Commands_topic_receive", 10)
        
    def publish(self, data):
        msg = String()
        msg.data = data
        logger.debug(f'Publishing: {data}')
        self.publisher_.publish(msg)

class Subscriber(Node):
    def __init__(self):    
        super().__init__("command_server_sub")
        self.subscription = self.create_subscription(String, "Commands_topic_send", self.listener_callback, 10)
        self.get_logger().info("Subscriber initialized ")
        self.flag = False
        self.info = ""

    def listener_callback(self,msg):
        logger.debug(f"Received message at listener : {msg.data}")
        self.flag = True
        self.info =  msg.data

class Server:
    def __init__(self, publisher, subscriber ):
        self.logger = logging.getLogger(__name__)
        self.websocket = None
        self.publisher = publisher
        self.subscriber = subscriber
        self.host = '0.0.0.0'
        self.port = 8082

        self.stop_server = False
    
    def kill(self):
        self.stop_server = True

    async def recv_loop(self, websocket):
        try:
            while not self.stop_server:
                message = await websocket.recv()
                self.logger.debug(f"Received message: {message}")
                if message != "":
                    self.publisher.publish(message)
        except Exception as e:
            self.logger.error(f"Recv loop error: {e}")

    async def send_loop(self, websocket):
        try:
            while not self.stop_server:
                if self.subscriber.flag and self.subscriber.info != "":
                    to_send = self.subscriber.info
                    await websocket.send(to_send)
                    self.subscriber.info = ""
                    self.subscriber.flag = False 
                await asyncio.sleep(0.01)  
        except Exception as e:
            self.logger.error(f"Send loop error: {e}")

    async def handler(self, websocket):
        self.logger.info(f"Client connected: {websocket.remote_address}")

        recv_task = asyncio.create_task(self.recv_loop(websocket))
        send_task = asyncio.create_task(self.send_loop(websocket))

        done, pending = await asyncio.wait(
            [recv_task, send_task], return_when=asyncio.FIRST_COMPLETED
        )                       

    async def start(self):

        async with websockets.serve( self.handler, self.host, self.port):
            self.logger.info(f"Server started on {self.host}:{self.port}")
            await asyncio.Future()

async def ros_spin_loop(executor):
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
        await asyncio.sleep(0.01)  

async def main (args=None):
    rclpy.init(args=args)
    pub = Publisher()
    sub = Subscriber()
    server = Server(publisher=pub, subscriber=sub)
    
    executor = SingleThreadedExecutor()
    executor.add_node(pub)
    executor.add_node(sub)

    ros_task = asyncio.create_task(ros_spin_loop(executor))
    server_task = asyncio.create_task(server.start())

    await asyncio.gather(ros_task, server_task)

if __name__ == "__main__":
    asyncio.run(main())
