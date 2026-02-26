#!/usr/bin/env python3
"""
object_detection_node.py
────────────────────────
Nodo ROS 2 (Humble) que:
  1. Recibe imágenes RGB + depth de la OAK-D
  2. Corre inferencia YOLOv8 (Ultralytics) en GPU Jetson
  3. Proyecta cada detección al espacio 3-D del mapa usando TF
  4. Publica MarkerArray para visualizar objetos en RViz / rtabmap_viz
  5. Guarda un JSON persistente con {label, posición 3D, timestamp, confianza}

Dependencias:
    pip install ultralytics opencv-python-headless --break-system-packages
    sudo apt install ros-humble-vision-msgs ros-humble-tf2-geometry-msgs

Integración con RTAB-Map:
  • Los marcadores se publican en el frame "map" → visibles en rtabmap_viz
  • El JSON se usa después por export_map_ply.py para anotar el PLY final
"""

import json
import math
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs  # noqa – necesario para transformar PointStamped


# ─────────────────────────────────────────────────────────────────────────────
# Colores por clase (BGR para OpenCV / RGBA para RViz)
# Amplía según las clases de tu modelo
# ─────────────────────────────────────────────────────────────────────────────
CLASS_COLORS_RVIZ: dict[str, tuple[float, float, float]] = {
    # label           R      G      B
    'person':        (1.0,  0.2,  0.2),
    'victim':        (1.0,  0.0,  0.0),
    'fire':          (1.0,  0.5,  0.0),
    'smoke':         (0.5,  0.5,  0.5),
    'hazmat':        (1.0,  1.0,  0.0),
    'door':          (0.0,  0.7,  1.0),
    'default':       (0.2,  1.0,  0.2),
}


class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('object_detection_node')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('model_path',     '/path/to/yolo/best.pt')
        self.declare_parameter('confidence_thr', 0.50)
        self.declare_parameter('iou_thr',        0.45)
        self.declare_parameter('device',         'cuda:0')
        self.declare_parameter('save_path',      '~/.ros/detected_objects.json')
        self.declare_parameter('map_frame',      'map')
        self.declare_parameter('camera_frame',   'oak-d-base-frame')
        self.declare_parameter('process_every_n', 5)   # 1 de cada N frames

        model_path    = self.get_parameter('model_path').value
        self.conf_thr = self.get_parameter('confidence_thr').value
        self.iou_thr  = self.get_parameter('iou_thr').value
        device        = self.get_parameter('device').value
        save_path     = self.get_parameter('save_path').value
        self.map_frame    = self.get_parameter('map_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.every_n      = self.get_parameter('process_every_n').value

        # ── Ruta de guardado ──────────────────────────────────────────────
        self.save_path = Path(save_path).expanduser()
        self.save_path.parent.mkdir(parents=True, exist_ok=True)
        self._load_existing_objects()

        # ── Modelo YOLO ───────────────────────────────────────────────────
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.model.to(device)
            self.get_logger().info(f'Modelo YOLO cargado: {model_path} en {device}')
        except Exception as e:
            self.get_logger().error(f'Error cargando YOLO: {e}')
            self.model = None

        # ── Internos ──────────────────────────────────────────────────────
        self.bridge        = CvBridge()
        self.camera_info   = None          # Se llena al recibir el primer mensaje
        self.frame_count   = 0
        self.marker_id_ctr = 0             # ID incremental para markers

        # TF listener
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Suscriptores sincronizados ─────────────────────────────────
        self._sub_info  = self.create_subscription(
            CameraInfo, '/right/camera_info',
            self._camera_info_cb, 1
        )
        sub_rgb   = Subscriber(self, Image, '/right/image_rect')
        sub_depth = Subscriber(self, Image, '/stereo/depth')

        self.sync = ApproximateTimeSynchronizer(
            [sub_rgb, sub_depth], queue_size=5, slop=0.05
        )
        self.sync.registerCallback(self._rgbd_callback)

        # ── Publicadores ──────────────────────────────────────────────────
        self.pub_markers = self.create_publisher(
            MarkerArray, '/detected_objects', 10
        )
        self.pub_debug_img = self.create_publisher(
            Image, '/detection_debug_image', 1
        )

        # Re-publicar marcadores acumulados cada 2 s
        self.create_timer(2.0, self._republish_all_markers)

        self.get_logger().info('ObjectDetectionNode listo.')

    # ─────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo):
        """Guarda intrínsecos de cámara (solo necesitamos el primero)."""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(
                f'Camera info recibida: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} '
                f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}'
            )

    def _rgbd_callback(self, rgb_msg: Image, depth_msg: Image):
        """Callback principal: corre YOLO y guarda objetos 3-D."""
        self.frame_count += 1
        if self.frame_count % self.every_n != 0:
            return
        if self.model is None or self.camera_info is None:
            return

        # Convertir mensajes → numpy
        try:
            rgb_img   = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'CvBridge error: {e}')
            return

        # depth_img puede ser float32 (metros) o uint16 (milímetros)
        if depth_img.dtype == np.uint16:
            depth_m = depth_img.astype(np.float32) / 1000.0
        else:
            depth_m = depth_img.astype(np.float32)

        # ── Inferencia YOLO ───────────────────────────────────────────
        results = self.model.predict(
            rgb_img,
            conf=self.conf_thr,
            iou=self.iou_thr,
            verbose=False,
            half=True,   # FP16 en GPU Jetson
        )

        new_markers: list[Marker] = []

        for r in results:
            for box in r.boxes:
                # Caja en píxeles
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf  = float(box.conf[0])
                cls_i = int(box.cls[0])
                label = self.model.names[cls_i]

                # Centro del bounding box
                cx_px = (x1 + x2) // 2
                cy_px = (y1 + y2) // 2

                # Profundidad robusta: mediana de parche 5×5 en el centro
                z_val = self._robust_depth(depth_m, cx_px, cy_px, patch=5)
                if z_val is None or z_val <= 0.1 or z_val > 10.0:
                    continue

                # Proyección 3-D en frame de cámara
                pt_cam = self._pixel_to_3d(cx_px, cy_px, z_val)

                # Transformar a frame "map" usando TF
                pt_map = self._transform_to_map(pt_cam, depth_msg.header)
                if pt_map is None:
                    continue

                # Construir objeto detectado
                obj = {
                    'id':         self.marker_id_ctr,
                    'label':      label,
                    'confidence': round(conf, 3),
                    'x':          round(pt_map.x, 4),
                    'y':          round(pt_map.y, 4),
                    'z':          round(pt_map.z, 4),
                    'timestamp':  rgb_msg.header.stamp.sec,
                    'frame':      self.map_frame,
                }

                # Guardar si no existe un objeto igual cerca (radio 0.3 m)
                if not self._is_duplicate(label, pt_map):
                    self.detected_objects.append(obj)
                    self._save_objects()
                    self.get_logger().info(
                        f'Objeto: {label} ({conf:.2f}) @ '
                        f'({pt_map.x:.2f}, {pt_map.y:.2f}, {pt_map.z:.2f}) m'
                    )

                # Marker RViz
                marker = self._make_marker(obj)
                new_markers.append(marker)
                self.marker_id_ctr += 1

                # Dibujar en imagen debug
                color_bgr = self._label_to_bgr(label)
                cv2.rectangle(rgb_img, (x1, y1), (x2, y2), color_bgr, 2)
                cv2.putText(
                    rgb_img,
                    f'{label} {conf:.2f} [{z_val:.2f}m]',
                    (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 1
                )

        # Publicar markers nuevos
        if new_markers:
            arr = MarkerArray()
            arr.markers = new_markers
            self.pub_markers.publish(arr)

        # Publicar imagen de debug (baja prioridad, compressed opcional)
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(rgb_img, 'bgr8')
            debug_msg.header = rgb_msg.header
            self.pub_debug_img.publish(debug_msg)
        except Exception:
            pass

    # ─────────────────────────────────────────────────────────────────────
    # Utilidades internas
    # ─────────────────────────────────────────────────────────────────────

    def _pixel_to_3d(self, u: int, v: int, z: float) -> Point:
        """Convierte (u, v, z) → Point en frame de cámara usando intrínsecos."""
        K = self.camera_info.k   # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]
        p = Point()
        p.x = (u - cx) * z / fx
        p.y = (v - cy) * z / fy
        p.z = z
        return p

    def _robust_depth(
        self, depth_m: np.ndarray, u: int, v: int, patch: int = 5
    ) -> float | None:
        """Mediana de un parche centrado en (u, v) para profundidad robusta."""
        h, w = depth_m.shape[:2]
        r = patch // 2
        u0, u1 = max(0, u - r), min(w, u + r + 1)
        v0, v1 = max(0, v - r), min(h, v + r + 1)
        region = depth_m[v0:v1, u0:u1]
        valid  = region[(region > 0.1) & np.isfinite(region)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _transform_to_map(self, pt_cam: Point, header) -> Point | None:
        """Transforma un Point del frame de cámara al frame 'map' usando TF2."""
        ps = PointStamped()
        ps.header = header
        ps.header.frame_id = self.camera_frame
        ps.point = pt_cam
        try:
            ps_map = self.tf_buffer.transform(ps, self.map_frame, timeout=rclpy.duration.Duration(seconds=0.1))
            return ps_map.point
        except Exception as e:
            self.get_logger().debug(f'TF error: {e}')
            return None

    def _is_duplicate(self, label: str, pt: Point, radius: float = 0.3) -> bool:
        """True si ya existe un objeto de la misma clase a menos de `radius` metros."""
        for obj in self.detected_objects:
            if obj['label'] != label:
                continue
            dx = obj['x'] - pt.x
            dy = obj['y'] - pt.y
            dz = obj['z'] - pt.z
            if math.sqrt(dx*dx + dy*dy + dz*dz) < radius:
                return True
        return False

    def _make_marker(self, obj: dict) -> Marker:
        """Crea un Marker de esfera + texto para RViz."""
        r, g, b = CLASS_COLORS_RVIZ.get(obj['label'], CLASS_COLORS_RVIZ['default'])

        # Esfera
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns     = 'objects'
        m.id     = obj['id'] * 2
        m.type   = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = obj['x']
        m.pose.position.y = obj['y']
        m.pose.position.z = obj['z']
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color  = ColorRGBA(r=r, g=g, b=b, a=0.85)
        m.lifetime.sec = 0   # permanente

        return m

    def _republish_all_markers(self):
        """Re-publica todos los objetos detectados para que no desaparezcan."""
        if not self.detected_objects:
            return
        arr = MarkerArray()
        for obj in self.detected_objects:
            arr.markers.append(self._make_marker(obj))
            # Label de texto
            arr.markers.append(self._make_text_marker(obj))
        self.pub_markers.publish(arr)

    def _make_text_marker(self, obj: dict) -> Marker:
        r, g, b = CLASS_COLORS_RVIZ.get(obj['label'], CLASS_COLORS_RVIZ['default'])
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns     = 'object_labels'
        m.id     = obj['id'] * 2 + 1
        m.type   = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = obj['x']
        m.pose.position.y = obj['y']
        m.pose.position.z = obj['z'] + 0.2
        m.pose.orientation.w = 1.0
        m.scale.z = 0.12
        m.color   = ColorRGBA(r=r, g=g, b=b, a=1.0)
        m.text    = f"{obj['label']}\n{obj['confidence']:.2f}"
        m.lifetime.sec = 0
        return m

    def _label_to_bgr(self, label: str) -> tuple[int, int, int]:
        r, g, b = CLASS_COLORS_RVIZ.get(label, CLASS_COLORS_RVIZ['default'])
        return (int(b * 255), int(g * 255), int(r * 255))

    def _load_existing_objects(self):
        """Carga sesión anterior si existe."""
        if self.save_path.exists():
            try:
                with open(self.save_path) as f:
                    self.detected_objects: list[dict] = json.load(f)
                self.marker_id_ctr = max((o['id'] for o in self.detected_objects), default=0) + 1
                self.get_logger().info(
                    f'Cargados {len(self.detected_objects)} objetos de sesión anterior.'
                )
            except Exception:
                self.detected_objects = []
        else:
            self.detected_objects = []

    def _save_objects(self):
        """Guarda la lista de objetos en JSON."""
        try:
            with open(self.save_path, 'w') as f:
                json.dump(self.detected_objects, f, indent=2)
        except Exception as e:
            self.get_logger().warn(f'No se pudo guardar JSON: {e}')


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
