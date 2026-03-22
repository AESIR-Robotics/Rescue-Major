import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TestTeleop(Node):
    def __init__(self):
        super().__init__('test_teleop')
        # Publicador conectado a Servo
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        # Enviar comandos 10 veces por segundo (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = TwistStamped()
        
        # --- EL TRUCO MAGICO: Poner la hora real exacta ---
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Mover hacia adelante en el eje X
        msg.twist.linear.x = 0.1
        msg.twist.linear.y = 0.1
        msg.twist.linear.z = 0.05
        
        self.publisher_.publish(msg)
        self.get_logger().info('Enviando velocidad con tiempo real activo...')

def main(args=None):
    rclpy.init(args=args)
    node = TestTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()