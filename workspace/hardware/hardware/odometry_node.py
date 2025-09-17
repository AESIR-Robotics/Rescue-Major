#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.subscription = self.create_subscription(Float32MultiArray, '/encoders', self.encoders_callback, 10)

    def encoders_callback(self, msg):
        encoder_izq, encoder_der = msg.data
        self.get_logger().info(f'Encoders: Izq: {encoder_izq}, Der: {encoder_der}')

def main():
    rclpy.init()
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
