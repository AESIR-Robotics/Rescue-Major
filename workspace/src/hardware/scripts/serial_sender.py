#!/usr/bin/env python3
"""
Serial Sender Node
Suscribe al topic /all_motors y envía los datos por USB serial sin modificaciones.
Formato esperado: "x,y,a,b,c,d" (string)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading


class SerialSenderNode(Node):
    def __init__(self):
        super().__init__('serial_sender')

        # Parámetros configurables
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('topic', '/all_motors')

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        topic_name = self.get_parameter('topic').get_parameter_value().string_value

        self.ser = None
        self.serial_available = False
        self.serial_lock = threading.Lock()

        # Intentar abrir el puerto serial
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1
            )
            self.serial_available = True
            self.get_logger().info(f"Serial abierto en {self.serial_port} @ {self.baud_rate}")
        except Exception as e:
            self.ser = None
            self.serial_available = False
            self.get_logger().warning(
                f"Serial no disponible en {self.serial_port}: {e}. Modo solo-lectura (imprime mensajes)."
            )

        # Suscripción al topic /all_motors
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.motors_callback,
            10
        )
        self.get_logger().info(f"Suscrito a {topic_name}")

    def motors_callback(self, msg: String):
        """
        Callback que recibe el string del topic y lo envía por serial.
        Formato: "x,y,a,b,c,d"
        """
        data = msg.data
        
        # Log del mensaje recibido (opcional, comentar para reducir spam)
        # self.get_logger().debug(f"Recibido: {data}")

        if self.serial_available and self.ser:
            try:
                with self.serial_lock:
                    # Enviar el string con terminador de línea
                    self.ser.write((data + '\n').encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Error enviando por serial: {e}")
        else:
            # Modo debug: imprimir en consola cuando no hay serial
            self.get_logger().info(f"[DEBUG] Enviar: {data}")

    def destroy_node(self):
        """Cerrar puerto serial al destruir el nodo."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info("Puerto serial cerrado")
            except Exception as e:
                self.get_logger().error(f"Error cerrando serial: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialSenderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
