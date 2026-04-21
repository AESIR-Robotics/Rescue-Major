#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelAmplifier(Node):
    def __init__(self):
        super().__init__('cmd_vel_amplifier')
        
        # Escuchamos a Nav2
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel', 
            self.listener_callback,
            10
        )
        
        # Publicamos hacia tu hardware real
        self.publisher = self.create_publisher(Twist, '/hardware_node/cmd_vel', 10)
        
        # --- LAS GANANCIAS (K) ---
        self.k_linear = -15.0
        self.k_angular = 1.0 

        self.get_logger().info(f"Amplificador Iniciado. Ganancia Lineal: {self.k_linear}, Angular: {self.k_angular}")

    def listener_callback(self, msg):
        amplified_msg = Twist()
        
        # Aplicamos la matemática de amplificación
        amplified_msg.linear.x = msg.linear.x * self.k_linear
        amplified_msg.angular.z = msg.angular.z * self.k_angular
        
        # Mantenemos los demás valores en cero (Skid-steer)
        amplified_msg.linear.y = 0.0
        amplified_msg.linear.z = 0.0
        amplified_msg.angular.x = 0.0
        amplified_msg.angular.y = 0.0
        
        self.publisher.publish(amplified_msg)

def main(args=None):
    rclpy.init(args=args)
    amplifier_node = CmdVelAmplifier()
    
    try:
        rclpy.spin(amplifier_node)
    except KeyboardInterrupt:
        pass
    finally:
        amplifier_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()