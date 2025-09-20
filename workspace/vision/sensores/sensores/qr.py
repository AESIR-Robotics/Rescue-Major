#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar import pyzbar
from datetime import datetime
import cv2
import os

class QRNode(Node):
    def __init__(self):
        super().__init__('qr_reader_node')
        self.bridge = CvBridge()
        self.images = set()
        self.count = 0
        self.image_path = "/home/ruy/qr_videos"
        os.makedirs(self.image_path, exist_ok=True)
        self.get_logger().info("QR Reader node started.")

        # Suscripción al tópico de la cámara
        self.subscription = self.create_subscription(
            Image,
            'cam0/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("Suscrito al tópico cam0/image_raw")

    def image_callback(self, msg):
        try:
            # Convertir el mensaje ROS Image a una imagen OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Decodificar códigos QR en la imagen
            font = cv2.FONT_HERSHEY_SIMPLEX
            barcodes = pyzbar.decode(frame)

            for barcode in barcodes:
                (x, y, w, h) = barcode.rect
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                barcodeData = barcode.data.decode("utf-8")
                barcodeType = barcode.type
                text = f"{barcodeData} ({barcodeType})"
                cv2.putText(frame, text, (x, y - 10), font, 0.5, (0, 0, 255), 2)

                if barcodeData not in self.images:
                    self.images.add(barcodeData)
                    self.take_picture(barcodeData, frame)

            # Mostrar la imagen procesada en tiempo real
            cv2.imshow("QR Reader", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Cerrando ventana de visualización")
                cv2.destroyAllWindows()

        except Exception as e:
            self.get_logger().error(f"Error en el callback de imagen: {str(e)}")

    def take_picture(self, name, frame):
        now = datetime.now().strftime("%Y%m%d-%H%M%S")
        img_name = f"{self.image_path}/image_{self.count}_{now}.jpg"
        txt_name = f"{self.image_path}/image_{self.count}_{now}.txt"
        self.count += 1

        cv2.imwrite(img_name, frame)
        with open(txt_name, "w") as f:
            f.write(name)
        self.get_logger().info(f"QR detectado y guardado: {name}")

def main(args=None):
    rclpy.init(args=args)
    node = QRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()