#!/usr/bin/env python3


#This script is for testing the publisher node and wifi connection 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('dummy_subscriber')
        self.get_logger().info('Dummy Subscriber Node has been started.')
        self.subscription = self.create_subscription(
            String,
            'Commands_topic_receive',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        print(f'Recibido: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()