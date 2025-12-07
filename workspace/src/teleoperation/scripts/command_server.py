#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import websockets 
from websockets import serve
import asyncio
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class Publisher(Node):
    def __init__(self):
        super().__init__("command_server_pub")
        self.vision_pub = self.create_publisher(String, "commands_for_vision", 10)
        self.dc_motors_pub = self.create_publisher(Float32MultiArray, "commands_for_dc_motors", 10)
        self.webrtc_pub = self.create_publisher(String, "commands_for_webrtc", 10)
        self.audio_pub = self.create_publisher(String, "audio_commands", 10)
        logger.info("Publisher node initialized")

    def publish(self, data):
        logger.info(f"Received message: {data}")
        topic = data.split(":")[0]

        if topic == "vision":
            msg = String()
            msg.data = data.split(":")[1]
            self.vision_pub.publish(msg)
            logger.info(f"Published vision command: {msg.data}")
        elif topic == "dc_motors":
            msg = Float32MultiArray()
            x, y = map(float, data.split(":")[1].split(","))
            right_motor = y + x
            left_motor = y - x
            msg.data = [right_motor, left_motor]
            self.dc_motors_pub.publish(msg)
            logger.info(f"Published motor command: left={left_motor:.2f}, right={right_motor:.2f}")
        elif topic == "webrtc":
            msg = String()
            msg.data = data.split(":")[1]
            self.webrtc_pub.publish(msg)
            logger.info(f"Published WebRTC command: {msg.data}")
        else:
            logger.warning(f"Unknown topic: {topic}")
    
class Subscriber(Node):
    def __init__(self):    
        super().__init__("command_server_sub")
        self.subscription = self.create_subscription(String, "send_to_client", self.listener_callback, 10)
        self.flag = False
        self.info = ""
        logger.info("Subscriber node initialized")

    def listener_callback(self, msg):
        logger.info(f"Received ROS message to send to client: {msg.data}")
        self.flag = True
        self.info = msg.data

class Server:
    def __init__(self, publisher, subscriber):
        self.publisher = publisher
        self.subscriber = subscriber
        self.host = '0.0.0.0'
        self.port = 8082
        self.stop_server = False
        self.connected_clients = set()
        logger.info(f"Server initialized on {self.host}:{self.port}")
    
    def kill(self):
        self.stop_server = True

    async def recv_loop(self, websocket, client_id):
        try:
            while not self.stop_server:
                message = await websocket.recv()
                logger.info(f"Received from client {client_id}: {message}")
                if message != "":
                    self.publisher.publish(message)
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Client {client_id} disconnected")
        except Exception as e:
            logger.error(f"Recv loop error for client {client_id}: {e}")

    async def send_loop(self, websocket, client_id):
        try:
            while not self.stop_server:
                if self.subscriber.flag and self.subscriber.info != "":
                    to_send = self.subscriber.info
                    logger.info(f"Sending to client {client_id}: {to_send}")
                    await websocket.send(to_send)
                    self.subscriber.info = ""
                    self.subscriber.flag = False 
                await asyncio.sleep(0.01)  
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Send loop: Client {client_id} disconnected")
        except Exception as e:
            logger.error(f"Send loop error for client {client_id}: {e}")

    async def handler(self, websocket):
        client_id = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        logger.info(f"New client connected: {client_id}")
        self.connected_clients.add(client_id)
        
        try:
            recv_task = asyncio.create_task(self.recv_loop(websocket, client_id))
            send_task = asyncio.create_task(self.send_loop(websocket, client_id))

            done, pending = await asyncio.wait(
                [recv_task, send_task], return_when=asyncio.FIRST_COMPLETED
            )
            
            # Cancel remaining tasks
            for task in pending:
                task.cancel()
                
        finally:
            self.connected_clients.discard(client_id)
            logger.info(f"Client {client_id} cleanup completed. Active clients: {len(self.connected_clients)}")

    async def start(self):
        logger.info("Starting WebSocket server...")
        async with websockets.serve(self.handler, self.host, self.port):
            logger.info(f"WebSocket server running on ws://{self.host}:{self.port}")
            await asyncio.Future()

async def ros_spin_loop(executor):
    logger.info("Starting ROS spin loop")
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
        await asyncio.sleep(0.01)  

async def main(args=None):
    logger.info("Initializing command server...")
    rclpy.init(args=args)
    pub = Publisher()
    sub = Subscriber()
    server = Server(publisher=pub, subscriber=sub)
    
    executor = SingleThreadedExecutor()
    executor.add_node(pub)
    executor.add_node(sub)

    logger.info("Starting server tasks...")
    ros_task = asyncio.create_task(ros_spin_loop(executor))
    server_task = asyncio.create_task(server.start())

    try:
        await asyncio.gather(ros_task, server_task)
    except KeyboardInterrupt:
        logger.info("Server shutting down...")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
