#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO  # Usamos la API oficial
import cv2
import numpy as np
import os


class YoloHazmatNode(Node):
    def __init__(self):
        super().__init__('yolo_hazmat_node')

        # Ruta absoluta al modelo
        model_path = '/home/ruy/ruy1_ws/src/sensores/sensores/best.pt'

        if not os.path.exists(model_path):
            self.get_logger().error(f'No se encontró el modelo en: {model_path}')
            return

        # Cargar el modelo YOLOv8 con Ultralytics
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # Suscripción al tópico de la cámara
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.get_logger().info('Nodo YOLO Hazmat iniciado y suscrito a /camera/image_raw')

    def image_callback(self, msg):
        try:
            # Convertir mensaje ROS Image a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Ejecutar la inferencia con el modelo YOLOv8
            results = self.model(cv_image)[0]  # YOLO devuelve una lista, tomamos el primer elemento

            # Iterar sobre las detecciones
            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf)
                cls = int(box.cls)
                label = self.model.names[cls] if cls in self.model.names else str(cls)

                self.get_logger().info(
                    f'Detectado: {label}, Conf: {conf:.2f}, Box: ({x1}, {y1}, {x2}, {y2})'
                )

        except Exception as e:
            self.get_logger().error(f'Error en callback de imagen: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloHazmatNode()
    if node:  # Solo correr si el nodo fue creado exitosamente (modelo cargado)
        rclpy.spin(node)
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
