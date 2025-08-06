#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')
        self.publisher_ = self.create_publisher(String, 'Commands_topic_send', 10)
        self.get_logger().info('Dummy Publisher Node has been started.')

    def publish_message(self):
        while rclpy.ok():
            user_input = input("Enter a message to publish: ")
            msg = String()
            msg.data = user_input
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = DummyPublisher()
    try:
        node.publish_message()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Dummy Publisher Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()