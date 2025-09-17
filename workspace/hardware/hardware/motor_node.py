#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.subscription = self.create_subscription(Int32MultiArray, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(Float32MultiArray, '/encoders', 10)
        self.timer = self.create_timer(.01, self.cmd_enc_callback)
        self.encoder_izq = 0
        self.encoder_der = 0

    def cmd_vel_callback(self, msg):
        motor_izq, motor_der = msg.data
        self.get_logger().info(f'Recibido: Izq: {motor_izq}, Der: {motor_der}')

    def cmd_enc_callback(self):
        self.encoder_izq = 1.5
        self.encoder_der = 2.7

        encoder_msg = Float32MultiArray()
        encoder_msg.data = [self.encoder_izq, self.encoder_der]
        self.publisher.publish(encoder_msg)

def main():
    rclpy.init()
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
Lo mismo que arriba pero con Serial

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
import serial  

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info("Conectado al puerto serial")
        except serial.SerialException:
            self.get_logger().error("No se pudo abrir el puerto serial")
            self.ser = None

        #recibir velocidades
        self.subscription = self.create_subscription(Int32MultiArray, '/cmd_vel', self.cmd_vel_callback, 10)

        #enviar valores de encoders
        self.publisher = self.create_publisher(Float32MultiArray, '/encoders', 10)

        #
        self.timer = self.create_timer(0.01, self.cmd_enc_callback)

        # variables de encoders
        self.encoder_izq = 0.0
        self.encoder_der = 0.0

    def cmd_vel_callback(self, msg):
        #Recibe las velocidades y las envía al microcontrolador vía serial
        motor_izq, motor_der = msg.data
        self.get_logger().info(f'Recibido: Izq: {motor_izq}, Der: {motor_der}')

        # Enviar valores al microcontrolador
        if self.ser:
            comando = f"{motor_izq},{motor_der}\n"
            self.ser.write(comando.encode('utf-8'))  # Enviar como string

    def cmd_enc_callback(self):
        #Lee los valores de los encoders desde el microcontrolador
        if self.ser:
            try:
                linea = self.ser.readline().decode('utf-8').strip()  # Leer una línea del puerto serial
                if linea:
                    valores = linea.split(',')
                    if len(valores) == 2:  # Verificar que haya dos valores
                        self.encoder_izq = float(valores[0])
                        self.encoder_der = float(valores[1])

                        # Publicar los valores de los encoders
                        encoder_msg = Float32MultiArray()
                        encoder_msg.data = [self.encoder_izq, self.encoder_der]
                        self.publisher.publish(encoder_msg)

            except Exception as e:
                self.get_logger().error(f"Error en la lectura del puerto serial: {e}")

def main():
    rclpy.init()
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()"""
