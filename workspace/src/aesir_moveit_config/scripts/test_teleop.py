#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
import sys
import select
import termios
import tty

# Mapeo de teclas: (linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
KEY_BINDINGS = {
    'w': ( 3.0,  0.0,  0.0,  0.0,  0.0,  0.0), # Adelante X
    's': (-3.0,  0.0,  0.0,  0.0,  0.0,  0.0), # Atrás X
    'a': ( 0.0,  3.0,  0.0,  0.0,  0.0,  0.0), # Izquierda Y
    'd': ( 0.0, -3.0,  0.0,  0.0,  0.0,  0.0), # Derecha Y
    'q': ( 0.0,  0.0,  3.0,  0.0,  0.0,  0.0), # Arriba Z
    'e': ( 0.0,  0.0, -3.0,  0.0,  0.0,  0.0), # Abajo Z
    'u': ( 0.0,  0.0,  0.0,  5.0,  0.0,  0.0), # Roll +
    'o': ( 0.0,  0.0,  0.0, -5.0,  0.0,  0.0), # Roll -
    'i': ( 0.0,  0.0,  0.0,  0.0,  5.0,  0.0), # Pitch +
    'k': ( 0.0,  0.0,  0.0,  0.0, -5.0,  0.0), # Pitch -
    'j': ( 0.0,  0.0,  0.0,  0.0,  0.0,  5.0), # Yaw +
    'l': ( 0.0,  0.0,  0.0,  0.0,  0.0, -5.0), # Yaw -
}

msg_info = """
---------------------------------------------
¡Controlador de Teclado Activo para Servo!
---------------------------------------------
Moviendo el Efector Final (Traslación):
   w : Adelante (+X)    s : Atrás (-X)
   a : Izquierda (+Y)   d : Derecha (-Y)
   q : Arriba (+Z)      e : Abajo (-Z)

Girando la Muñeca (Rotación):
   u / o : Roll (+/-)
   i / k : Pitch (+/-)
   j / l : Yaw (+/-)

Espaciadora o cualquier otra tecla : PARAR
CTRL-C para salir
---------------------------------------------
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
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

        # Configuración para leer el teclado en Linux
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed_multiplier = 0.2  # Escala de velocidad (0.2 m/s o rad/s)

        self.get_logger().info('Nodo de teleoperación iniciado.')
        print(msg_info)

        # Correr el bucle de lectura a ~10Hz
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
        """Lee una sola tecla de la terminal sin bloquear el programa."""
        tty.setraw(sys.stdin.fileno())
        # Espera un máximo de 0.1 segundos por una tecla
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_callback(self):
        key = self.get_key()
        
        # Si presionamos Ctrl+C, salir
        if key == '\x03':
            rclpy.shutdown()
            return

        x = y = z = th_x = th_y = th_z = 0.0

        if key in KEY_BINDINGS:
            # Extraer las velocidades del diccionario
            x, y, z, th_x, th_y, th_z = KEY_BINDINGS[key]
        elif key != '':
            # Si presiona barra espaciadora u otra tecla, se queda todo en 0 (Frenar)
            pass 

        # Crear el mensaje
        msg = TwistStamped()
        
        # EL TRUCO MÁGICO: Sello de tiempo real vital para Servo
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Aplicar el multiplicador de velocidad
        msg.twist.linear.x = x * self.speed_multiplier
        msg.twist.linear.y = y * self.speed_multiplier
        msg.twist.linear.z = z * self.speed_multiplier
        msg.twist.angular.x = th_x * self.speed_multiplier
        msg.twist.angular.y = th_y * self.speed_multiplier
        msg.twist.angular.z = th_z * self.speed_multiplier
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    # Quick fix: Save the original terminal settings safely here so the 'finally' block can properly restore them
    original_settings = termios.tcgetattr(sys.stdin)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Restaurar la terminal al salir de forma segura
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()