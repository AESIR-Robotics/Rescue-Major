#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Import your custom message exactly as your C++ plugin defines it
from hardware.msg import JointControl 

class LoopbackSimulator(Node):
    def __init__(self):
        super().__init__('hardware_loopback_node')
        
        # Publisher: Send the "fake" physical encoder data back to the C++ plugin
        self.feedback_pub = self.create_publisher(JointState, '/hardware_feedback', 10)
        
        # Subscriber: Listen to the commands your C++ plugin is sending out
        self.command_sub = self.create_subscription(
            JointControl,
            '/commands_hardware',
            self.command_callback,
            10
        )
        self.get_logger().info('Loopback active: Mirroring /commands_hardware -> /hardware_feedback')

    def command_callback(self, cmd_msg):
        # Create a standard ROS 2 JointState message
        state_msg = JointState()
        
        # Give it a fresh timestamp so MoveIt knows it's current
        state_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Map your custom JointControl arrays directly to the standard JointState arrays
        state_msg.name = cmd_msg.joint_names
        state_msg.position = cmd_msg.position
        state_msg.velocity = cmd_msg.velocity
        state_msg.effort = cmd_msg.effort
        
        # Publish the simulated physical hardware feedback!
        self.feedback_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LoopbackSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()