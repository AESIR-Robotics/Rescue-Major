#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import hashlib

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from depthai_ros_msgs.msg import SpatialDetectionArray
from rtabmap_msgs.msg import LandmarkDetection, LandmarkDetections

import tf2_ros
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import sensor_msgs_py.point_cloud2 as pc2

CLASS_COLORS = {
    'person':   (1.0, 0.2, 0.2),
    'victim':   (1.0, 0.0, 0.0),
    'fire':     (1.0, 0.5, 0.0),
    'default':  (0.8, 0.8, 0.0),
}

LIDAR_CONE_ANGLE_RAD = np.deg2rad(8.0)
LIDAR_MAX_RANGE = 15.0
MIN_CONFIDENCE = 0.1  # Bajado casi a 0 para capturar todos los rangos crudos de YOLO

def class_to_int_id(class_label: str, spatial_x: float, spatial_y: float) -> int:
    key = f"{class_label}_{round(spatial_x, 1)}_{round(spatial_y, 1)}"
    return int(hashlib.md5(key.encode()).hexdigest()[:6], 16) % 100000

def quat_to_mat(q):
    # Matemáticas puras: Evita el colapso de tf2_geometry_msgs
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])

class LandmarkTFPublisher(Node):
    def __init__(self):
        super().__init__('landmark_tf_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_detections = self.create_subscription(
            SpatialDetectionArray, '/color/yolov4_Spatial_detections', self.detections_callback, 10)
        self.sub_cloud = self.create_subscription(
            PointCloud2, '/unilidar/cloud', self.cloud_callback, 5)

        self.pub_landmarks = self.create_publisher(LandmarkDetections, '/landmark_detections', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/yolo_landmark_markers', 10)

        self.latest_cloud_points = None
        self.known_landmarks = {}
        self.marker_id_counter = 0

    def cloud_callback(self, msg: PointCloud2):
        try:
            points_generator = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            pts_list = [[float(p[0]), float(p[1]), float(p[2])] for p in points_generator]

            if pts_list:
                pts_array = np.array(pts_list, dtype=np.float32)
                dists = np.linalg.norm(pts_array, axis=1)
                self.latest_cloud_points = pts_array[dists < LIDAR_MAX_RANGE]
        except Exception as e:
            pass

    def detections_callback(self, msg: SpatialDetectionArray):
        if not msg.detections: return

        ld_msg = LandmarkDetections()
        ld_msg.header.stamp = msg.header.stamp
        ld_msg.header.frame_id = 'unilidar_lidar'
        marker_array = MarkerArray()

        for det in msg.detections:
            if not det.results: continue
            best = max(det.results, key=lambda r: r.score)
            if best.score < MIN_CONFIDENCE: continue

            class_label = str(best.class_id)
            cam_x, cam_y, cam_z = det.position.x, det.position.y, det.position.z

            if cam_z <= 0.1 or np.isnan(cam_z): continue

            ref_z = self._refine_depth(cam_x, cam_y, cam_z)
            scale = ref_z / cam_z if cam_z > 0 else 1.0
            
            pos_in_lidar = self._transform_to_lidar(cam_x * scale, cam_y * scale, ref_z, msg.header.frame_id)
            if pos_in_lidar is None: continue

            lm_id = class_to_int_id(class_label, pos_in_lidar[0], pos_in_lidar[1])

            if self._is_duplicate(lm_id, pos_in_lidar):
                self._publish_tf(lm_id, class_label, pos_in_lidar, msg.header.stamp)
                continue

            self.known_landmarks[lm_id] = pos_in_lidar
            self.get_logger().info(f'VÍCTIMA DETECTADA [Clase {class_label}] a {ref_z:.2f}m. Score: {best.score:.1f}')

            self._publish_tf(lm_id, class_label, pos_in_lidar, msg.header.stamp)
            
            # --- Corrección Estructural Absoluta del Mensaje ---
            lm = LandmarkDetection()
            lm.header.stamp = msg.header.stamp
            lm.header.frame_id = 'unilidar_lidar'
            lm.landmark_frame_id = f'lm_{class_label}_{lm_id}'
            lm.id = lm_id
            lm.size = 0.3
            lm.pose.pose.position.x = float(pos_in_lidar[0])
            lm.pose.pose.position.y = float(pos_in_lidar[1])
            lm.pose.pose.position.z = float(pos_in_lidar[2])
            lm.pose.pose.orientation.w = 1.0
            cov = [0.0] * 36
            cov[0], cov[7], cov[14] = 0.05, 0.05, 0.10
            cov[21], cov[28], cov[35] = 0.01, 0.01, 0.01
            lm.pose.covariance = cov
            
            ld_msg.landmarks.append(lm)

            color = CLASS_COLORS.get(class_label.lower(), CLASS_COLORS['default'])
            
            # 1. Esfera en 3D
            m = Marker()
            m.header.stamp = msg.header.stamp
            m.header.frame_id = 'unilidar_lidar'
            m.ns, m.id = f'lm_{class_label}', self.marker_id_counter
            m.type, m.action = Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(pos_in_lidar[0]), float(pos_in_lidar[1]), float(pos_in_lidar[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], 0.8
            marker_array.markers.append(m)
            self.marker_id_counter += 1

            # 2. Texto flotante con Clase y Score Crudo
            t = Marker()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'unilidar_lidar'
            t.ns, t.id = f'txt_{class_label}', self.marker_id_counter
            t.type, t.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x = float(pos_in_lidar[0])
            t.pose.position.y = float(pos_in_lidar[1])
            t.pose.position.z = float(pos_in_lidar[2]) + 0.4
            t.pose.orientation.w = 1.0
            t.scale.z = 0.2
            t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 1.0, 1.0, 1.0
            t.text = f'Clase: {class_label}\nScore: {best.score:.1f}'
            marker_array.markers.append(t)
            self.marker_id_counter += 1

        if ld_msg.landmarks: self.pub_landmarks.publish(ld_msg)
        if marker_array.markers: self.pub_markers.publish(marker_array)

    def _refine_depth(self, cx, cy, cz):
        if self.latest_cloud_points is None or len(self.latest_cloud_points) < 10: return cz
        try:
            dir_cam = np.array([cx, cy, cz], dtype=np.float64)
            dir_cam /= np.linalg.norm(dir_cam)
            tf = self.tf_buffer.lookup_transform('unilidar_lidar', 'oak_rgb_camera_optical_frame', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05))
            rot = quat_to_mat(tf.transform.rotation)
            dir_lidar = rot @ dir_cam
            dir_lidar /= np.linalg.norm(dir_lidar)
            pts = self.latest_cloud_points
            mask = ((pts / (np.linalg.norm(pts, axis=1, keepdims=True) + 1e-6)) @ dir_lidar) > np.cos(LIDAR_CONE_ANGLE_RAD)
            pts_in_cone = pts[mask]
            if len(pts_in_cone) < 3: return cz
            median_d = float(np.median(np.linalg.norm(pts_in_cone, axis=1)))
            return median_d if abs(median_d - cz) <= 2.0 else cz
        except: return cz

    def _transform_to_lidar(self, x, y, z, frame):
        try:
            tf = self.tf_buffer.lookup_transform('unilidar_lidar', frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05))
            v = np.array([float(x), float(y), float(z)])
            rot = quat_to_mat(tf.transform.rotation)
            trans = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
            p_lidar = rot @ v + trans
            return p_lidar
        except Exception as e:
            self.get_logger().warn(f"TF Lookup failed: {e}")
            return None

    def _publish_tf(self, l_id, cls, pos, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'unilidar_lidar'
        t.child_frame_id = f'lm_{cls}_{l_id}'
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _is_duplicate(self, l_id, pos):
        if l_id in self.known_landmarks:
            if np.linalg.norm(pos - self.known_landmarks[l_id]) < 0.8: return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = LandmarkTFPublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()