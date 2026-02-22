#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import threading

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # Configuración Serial (ajusta el puerto según tu Raspberry/PC)
        self.serial_port = "/dev/ttyACM0"   # o "/dev/serial0" en la Raspberry
        self.baud_rate = 115200
        self.ser = None
        self.serial_available = False

        # Intentar abrir el puerto serial; si falla, seguir en modo solo-lectura
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.serial_available = True
            self.get_logger().info(f"Serial abierto en {self.serial_port} @ {self.baud_rate}")
        except Exception as e:
            self.ser = None
            self.serial_available = False
            self.get_logger().warning(
                f"Dispositivo serial no disponible en {self.serial_port}: {e}. Ejecutando en modo lectura (solo imprime topic)."
            )

        # Lock para acceso seguro al puerto serie (aunque no esté disponible)
        self.serial_lock = threading.Lock()

        # recibir velocidades
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/dc_motors',
            self.cmd_vel_callback,
            10
        )


    def motors(self, left, right):
        if left < 0:
            left = ((left) * -1) + 100
        if right < 0:
            right = ((right) * -1) + 100

        data = [(11, left), (12, right)]
        for command, value in data:
            self.send_data(command, value)

    def send_data(self, command, number):
        # Si no hay serial disponible, no lanzar error: solo informar qué se habría enviado
        if not self.serial_available or self.ser is None:
            self.get_logger().info(f"(No serial) Se intentó enviar - cmd: {command}, valor: {number}")
            return

        try:
            data_to_send = bytearray([
                command,
                (number >> 8) & 0xFF,  # High byte
                number & 0xFF          # Low byte
            ])

            with self.serial_lock:
                self.ser.write(data_to_send)

        except Exception as e:
            self.get_logger().error(f"Error sending data: {e}")

    def cmd_vel_callback(self, msg):
        # Recibe las velocidades y las envía al microcontrolador vía serial
        motor_der, motor_izq = msg.data
        self.get_logger().info(f'Recibido: Izq: {motor_izq}, Der: {motor_der}')
        # Siempre imprimir lo recibido. Solo enviar a los motores si el serial está disponible.
        if self.serial_available:
            self.motors(int(motor_izq * 100), int(motor_der * 100))

def main():
    rclpy.init()
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
import smbus2  
import threading

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # I2C Configuration
        self.I2C_SLAVE_ADDRESS = 0x09  # Address of the Teensy
        self.bus = smbus2.SMBus(1)
        # Create a Lock object to manage access to the I2C bus
        self.i2c_lock = threading.Lock()

        #recibir velocidades
        self.subscription = self.create_subscription(Float32MultiArray, 'commands_for_dc_motors', self.cmd_vel_callback, 10)

        #enviar valores de encoders
        self.publisher = self.create_publisher(Float32MultiArray, '/encoders', 10)

        #
        self.timer = self.create_timer(0.01, self.cmd_enc_callback)

        # variables de encoders
        self.encoder_izq = 0.0
        self.encoder_der = 0.0
    
    def motors(self, left, right):
        if left < 0:
            left = ((left) * -1) + 100
        if right < 0:
            right = ((right) * -1) + 100
            
        data = [(11, left), (12, right)]
        for command, value in data:
            self.send_data(command, value)
    
    def send_data(self,command, number):
        try:
            data_to_send = [command,
                            (number >> 8) & 0xFF,  # High byte
                            number & 0xFF]         # Low byte
            
            with self.i2c_lock:
                self.bus.write_i2c_block_data(self.I2C_SLAVE_ADDRESS, command, data_to_send[1:])
        except Exception as e:
            att = 0
                #print(f"Error sending data: {e}")

    def cmd_vel_callback(self, msg):
        #Recibe las velocidades y las envía al microcontrolador vía serial
        motor_der, motor_izq = msg.data
        self.get_logger().info(f'Recibido: Izq: {motor_izq}, Der: {motor_der}')
        self.motors(int(motor_izq * 100 ), int(motor_der * 100))

def main():
    rclpy.init()
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""