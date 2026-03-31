#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger  # <-- Import added for the service
import sys
import select
import termios
import tty

# Mapping keys to (joint_name, direction_multiplier)
KEY_BINDINGS = {
    'q': ('joint_1',  3.0), 'a': ('joint_1', -3.0),
    'w': ('joint_2',  3.0), 's': ('joint_2', -3.0),
    'e': ('joint_3',  3.0), 'd': ('joint_3', -3.0),
    'r': ('joint_4',  3.0), 'f': ('joint_4', -3.0),
    't': ('joint_5',  3.0), 'g': ('joint_5', -3.0),
    'y': ('joint_6',  3.0), 'h': ('joint_6', -3.0),
}

msg_info = """
---------------------------------------------
¡Safe Joint Keyboard Controller Active!
---------------------------------------------
Control individual joints (Radians/sec):

  J1:  [Q] (+) / [A] (-)
  J2:  [W] (+) / [S] (-)
  J3:  [E] (+) / [D] (-)
  J4:  [R] (+) / [F] (-)
  J5:  [T] (+) / [G] (-)
  J6:  [Y] (+) / [H] (-)

Spacebar or any other key : STOP ALL JOINTS
CTRL-C to quit
---------------------------------------------
"""

class SafeJointKeyboard(Node):
    def __init__(self):
        super().__init__('safe_joint_keyboard')
        
        # Publish to Servo's joint command topic
        self.publisher_ = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        
        # --- NEW SERVICE CALL LOGIC ---
        self.cli = self.create_client(Trigger, '/servo_node/start_servo')
        self.get_logger().info('Esperando al servicio /servo_node/start_servo...')
        
        # Wait until the service is up
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, esperando de nuevo...')
            
        self.get_logger().info('Servicio encontrado. Iniciando Servo...')
        self.req = Trigger.Request()
        
        # Call the service asynchronously so it doesn't block the node
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.start_servo_callback)
        # ------------------------------

        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0.5  # Base speed in radians per second

        self.get_logger().info('Safe Joint Teleop node started.')
        print(msg_info)

        # Run the loop at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def start_servo_callback(self, future):
        """Callback to handle the response from the start_servo service call."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Éxito al iniciar servo: {response.message}')
            else:
                self.get_logger().warn(f'Servo reportó fallo al iniciar: {response.message}')
        except Exception as e:
            self.get_logger().error(f'La llamada al servicio falló: {e}')

    def get_key(self):
        """Read a single keypress without blocking."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_callback(self):
        key = self.get_key()
        
        if key == '\x03':  # Ctrl+C
            rclpy.shutdown()
            return

        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if key in KEY_BINDINGS:
            # If a valid key is pressed, move that specific joint
            joint_name, direction = KEY_BINDINGS[key]
            msg.joint_names = [joint_name]
            msg.velocities = [self.speed * direction]
        else:
            # If no key (or an invalid key) is pressed, actively stop all joints
            msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            msg.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafeJointKeyboard()
    
    # Save the original terminal settings safely here
    original_settings = termios.tcgetattr(sys.stdin)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Restore the terminal safely on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()