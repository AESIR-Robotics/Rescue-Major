#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar import pyzbar
from ultralytics import YOLO
import cv2
from datetime import datetime


class MultiDetectionNode(Node):
    def __init__(self):
        super().__init__('multi_detection_node')

        # Inicialización de variables
        self.bridge = CvBridge()
        self.mode = 0  # 0: Ninguna detección, 1: QR, 2: Hazmat, 3: Movimiento
        self.camera = 0

        # 📌 Usar rutas relativas al script
        script_dir = os.path.dirname(os.path.realpath(__file__))

        # Ruta al modelo
        model_path = os.path.join(script_dir, "best.pt")
        if not os.path.exists(model_path):
            self.get_logger().error(f"⚠️ No se encontró el modelo en: {model_path}")
            self.model = None
        else:
            self.model = YOLO(model_path)
            self.get_logger().info(f"✅ Modelo cargado desde: {model_path}")

        # Carpeta para guardar imágenes
        self.image_path = os.path.join(script_dir, "qr_videos")
        os.makedirs(self.image_path, exist_ok=True)
        self.get_logger().info(f"📁 Carpeta de imágenes: {self.image_path}")

        self.images = set()
        self.count = 0
        self.prev_frame = None

        # Suscripción al tópico de la cámara
        self.subscription = self.create_subscription(
            Image,
            'cam0/image_raw',
            self.image_callback,
            10
        )

        # Suscripción para recibir comandos
        self.command_subscriber = self.create_subscription(
            String,
            "Commands_topic_receive",
            self.command_callback,
            10
        )

        self.get_logger().info("Nodo MultiDetection iniciado y escuchando Commands_topic_receive")

    # Callback para recibir mensajes de modo
    def command_callback(self, msg):
        """
        Espera mensajes en formato:
        vision:mode,<camara>,<numero_de_modo>
        Ejemplo: "vision:mode,0,2"
        """
        self.get_logger().info(f"Mensaje recibido: {msg.data}")

        if not msg.data.startswith("vision:mode"):
            self.get_logger().debug("Mensaje ignorado: no es para vision.")
            return

        try:
            _, payload = msg.data.split(":", 1)
            parts = payload.split(",")

            if len(parts) < 3:
                self.get_logger().warn("Formato incorrecto. Se esperaban 3 valores.")
                return

            self.camera = int(parts[1])
            new_mode = int(parts[2])

            if new_mode < 0 or new_mode > 3:
                self.get_logger().warn(f"Modo {new_mode} fuera de rango (0-3).")
                return

            self.mode = new_mode
            self.get_logger().info(f"Modo actualizado a {self.mode} (cámara {self.camera})")

        except ValueError:
            self.get_logger().error(f"No se pudo convertir el modo '{parts[2]}' a número.")
        except Exception as e:
            self.get_logger().error(f"Error procesando mensaje: {e}")

    # Callback para procesar la imagen según el modo
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.mode == 1:
                frame = self.detect_qr(frame)
            elif self.mode == 2 and self.model:
                frame = self.detect_hazmat(frame)
            elif self.mode == 3:
                frame = self.detect_motion(frame)

            cv2.imshow("Multi Detection Viewer", frame)
            key = cv2.waitKey(1) & 0xFF

            # Cambio manual de modo con el teclado (opcional)
            if key == ord('1'):
                self.mode = 1
                self.get_logger().info("Modo: Detección de QR (manual)")
            elif key == ord('2'):
                self.mode = 2
                self.get_logger().info("Modo: Detección de Hazmat (manual)")
            elif key == ord('3'):
                self.mode = 3
                self.get_logger().info("Modo: Detección de Movimiento (manual)")
            elif key == ord('0'):
                self.mode = 0
                self.get_logger().info("Modo: Ninguna detección (manual)")

        except Exception as e:
            self.get_logger().error(f"Error en el callback de imagen: {str(e)}")

    # Funciones de detección
    def detect_qr(self, frame):
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

        return frame

    def detect_hazmat(self, frame):
        results = self.model(frame)[0]
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf)
            cls = int(box.cls)
            label = self.model.names[cls] if cls in self.model.names else str(cls)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{label} {conf:.2f}', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return frame

    def detect_motion(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        if self.prev_frame is None:
            self.prev_frame = gray
            return frame

        diff = cv2.absdiff(self.prev_frame, gray)
        _, thresh = cv2.threshold(diff, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)

        contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < 10000:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        self.prev_frame = gray
        return frame

    # Guardar QR detectado
    def take_picture(self, name, frame):
        now = datetime.now().strftime("%Y%m%d-%H%M%S")
        img_name = os.path.join(self.image_path, f"image_{self.count}_{now}.jpg")
        txt_name = os.path.join(self.image_path, f"image_{self.count}_{now}.txt")
        self.count += 1
        cv2.imwrite(img_name, frame)
        with open(txt_name, "w") as f:
            f.write(name)
        self.get_logger().info(f"QR detectado y guardado: {name}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
