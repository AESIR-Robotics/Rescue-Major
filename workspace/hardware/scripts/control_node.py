#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.publisher = self.create_publisher(Int32MultiArray, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_speeds)

    def publish_speeds(self):
        msg = Int32MultiArray()
        msg.data = [0, -30]  # Velocidades simuladas
        self.get_logger().info(f'Publicando velocidades: Izq: {msg.data[0]}, Der: {msg.data[1]}')
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
