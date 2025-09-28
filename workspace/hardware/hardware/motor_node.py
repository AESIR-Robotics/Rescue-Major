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

        # # Enviar valores al microcontrolador
        # if self.ser:
        #     comando = f"{motor_izq},{motor_der}\n"
        #     self.ser.write(comando.encode('utf-8'))  # Enviar como string

    def cmd_enc_callback(self):
        # #Lee los valores de los encoders desde el microcontrolador
        # if self.ser:
        #     try:
        #         linea = self.ser.readline().decode('utf-8').strip()  # Leer una línea del puerto serial
        #         if linea:
        #             valores = linea.split(',')
        #             if len(valores) == 2:  # Verificar que haya dos valores
        #                 self.encoder_izq = float(valores[0])
        #                 self.encoder_der = float(valores[1])

        #                 # Publicar los valores de los encoders
        #                 encoder_msg = Float32MultiArray()
        #                 encoder_msg.data = [self.encoder_izq, self.encoder_der]
        #                 self.publisher.publish(encoder_msg)

        #     except Exception as e:
        #         self.get_logger().error(f"Error en la lectura del puerto serial: {e}")
        pass

def main():
    rclpy.init()
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        