#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision.srv import Command
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
        self.threshold = 20 # senbility_dectetion_motion

        # 📌 Usar rutas relativas al script
        script_dir = os.path.dirname(os.path.realpath(__file__))

        # Ruta al modelo
        model_path = os.path.join(script_dir, "best.pt")
        if not os.path.exists(model_path):
            self.get_logger().error(f"No se encontró el modelo en: {model_path}")
            self.model = None
        else:
            #self.model = YOLO(model_path).to('cuda')
            self.model = YOLO(model_path)
            self.get_logger().info(f"Modelo cargado desde: {model_path}")

        # Carpeta para guardar imágenes
        self.image_path = os.path.join(script_dir, "../../../saves/vision")
        os.makedirs(self.image_path, exist_ok=True)
        self.get_logger().info(f"Carpeta de imágenes: {self.image_path}")

        self.images = set()
        self.count_qr = 0
        self.count_hazmat = 0
        self.detected_hazmats = set()
        self.prev_frame = None

        # Publisher al topico del video hacia la interfaz
        self.publisher = self.create_publisher(Image, 'cam_sensors/image', 10)

        # Suscripción al tópico de la cámara
        self.subscription = self.create_subscription(
            Image,
            'cam_sensors/image_raw',
            self.image_callback,
            10
        )

        # Suscripción para recibir comandos
        self.srv = self.create_service(
            Command,
            'command_vision_sensors',
            self.command_callback
        )

        self.get_logger().info("Nodo MultiDetection iniciado y escuchando Commands_topic_receive")

    # Callback para recibir mensajes de modo
    def command_callback(self, request, response):
        """
        Espera mensajes en formato:
        vision:mode,<numero_de_modo>
        Ejemplo: "vision:mode,2"
        vision:threshold,<numero>
        Ejemplo: vision: "vision:threshold,20"
        """
        cmd = request.data
        self.get_logger().info(f"Mensaje recibido: {cmd}")

        try:
            if cmd.lower().startswith("vision"):
                cmd = cmd[7:]

            cmd = cmd.replace(",",":")

            parts = cmd.split(":")
            if len(parts) < 2:
                response.success = False
                response.message = "Formato inválido. Use 'vision:mode,1'"
                return response

            label = parts[0].strip().lower()
            value_str = parts[1].strip()
            value = int(value_str)

            # Lógica para MODO
            if label == "mode":
                if 0 <= value <= 3:
                    self.mode = value
                    response.success = True
                    response.message = f"Modo de detección actualizado a: {self.mode}"
                else:
                    response.success = False
                    response.message = "Error: Modo fuera de rango (0-3)"

            # Lógica para THRESHOLD (Sensibilidad de movimiento)
            elif label == "threshold":
                if 0 <= value <= 255:
                    self.threshold = value
                    response.success = True
                    response.message = f"Umbral de movimiento ajustado a: {self.threshold}"
                else:
                    response.success = False
                    response.message = "Error: Threshold fuera de rango (0-255)"

            else:
                response.success = False
                response.message = f"Comando '{label}' no reconocido"

        except ValueError:
            response.success = False
            response.message = "Error: El valor debe ser un número entero"
        except Exception as e:
            response.success = False
            response.message = f"Error inesperado: {str(e)}"

        return response

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

            # Show image
            #cv2.imshow("Multi Detection Viewer", frame)
            key = cv2.waitKey(1) & 0xFF

            frame = self.bridge.cv2_to_imgmsg(frame, "bgr8")

            self.publisher.publish(frame)

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
                self.take_picture(barcodeData, frame, "qr")

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
            
            if label not in self.detected_hazmats:
                self.take_picture(label, frame, "hazmat")
                self.detected_hazmats.add(label)
        return frame

    def detect_motion(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        if self.prev_frame is None:
            self.prev_frame = gray
            return frame

        diff = cv2.absdiff(self.prev_frame, gray)
        _, thresh = cv2.threshold(diff, self.threshold, 255, cv2.THRESH_BINARY)
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
    def take_picture(self, name, frame, file):
        now = datetime.now().strftime("%Y%m%d-%H%M%S")

        if file == "qr":
            self.get_logger().info(self.image_path)
            image_path = os.path.join(self.image_path, "qr_images")
            count = self.count_qr
            self.count_qr += 1
        elif file == "hazmat":
            self.get_logger().info(self.image_path)
            image_path = os.path.join(self.image_path, "hazmat_images")
            count = self.count_hazmat
            self.count_hazmat += 1

        img_name = os.path.join(image_path, f"image_{count}_{now}.jpg")
        txt_name = os.path.join(image_path, f"image_{count}_{now}.txt")
        cv2.imwrite(img_name, frame)
        with open(txt_name, "w") as f:
            f.write(name)
        self.get_logger().info(f"Detectado y guardado: {name}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()