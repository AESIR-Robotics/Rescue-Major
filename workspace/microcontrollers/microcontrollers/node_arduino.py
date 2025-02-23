import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class Arduino(Node):
    def __init__(self):
        super().__init__('arduino')

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'arduino_input',
            self.listener_callback,
            10)  # frequency (Hz)
        self.subscription  # Prevent unused variable warning

        # Publisher
        self.publisher = self.create_publisher(String, 'arduino_output', 10)

        # Data arduino
        self.json_dir = "resources/commands_json"
        self.baudRate = 9600
        self.serialDevice = "/dev/ttyUSB0"
        self.arduino = None
        self.arduino_message = String()

        # Connect to Arduino
        self.arduino_connect()

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        self.arduino_message.data = msg.data  # Update the message
        self.publisher.publish(self.arduino_message)
        self.get_logger().info(f'Published: "{self.arduino_message.data}"')

    def arduino_connect(self):
        try:
            self.arduino = serial.Serial(self.serialDevice, self.baudRate)
            self.arduino.dsrdtr = True
            self.arduino.flushInput()
            self.arduino.flushOutput()
            self.get_logger().info("Arduino connected successfully")
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.arduino = None

    def arduino_read(self):
        if self.arduino and self.arduino.in_waiting > 0:
            self.arduino_message.data = str(self.arduino.readline())
            self.arduino_message.data = self.arduino_message.data[2:][:-5]  # Fix indexing

def main(args=None):
    rclpy.init(args=args)
    node = Arduino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()