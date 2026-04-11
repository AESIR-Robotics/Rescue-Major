#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

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
MIN_CONFIDENCE = 0.1
MIN_DIST_BETWEEN_VICTIMS = 2.0  # Rango de fusión para no saturar

def quat_to_mat(q):
    """ Convierte cuaterniones a matriz de rotación 3x3 usando Numpy puro """
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return np.array([
        [1.0 - 2.0*y*y - 2.0*z*z,     2.0*x*y - 2.0*z*w,     2.0*x*z + 2.0*y*w],
        [    2.0*x*y + 2.0*z*w, 1.0 - 2.0*x*x - 2.0*z*z,     2.0*y*z - 2.0*x*w],
        [    2.0*x*z - 2.0*y*w,     2.0*y*z + 2.0*x*w, 1.0 - 2.0*x*x - 2.0*y*y]
    ], dtype=np.float64)

class LandmarkTFPublisher(Node):
    def __init__(self):
        super().__init__('landmark_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Suscripciones
        self.sub_detections = self.create_subscription(
            SpatialDetectionArray, '/color/yolov4_Spatial_detections', self.detections_callback, 10)
        self.sub_cloud = self.create_subscription(
            PointCloud2, '/unilidar/cloud', self.cloud_callback, 5)

        # Publicadores
        self.pub_landmarks = self.create_publisher(LandmarkDetections, '/landmark_detections', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/yolo_landmark_markers', 10)

        self.latest_cloud_points = None
        self.landmark_id_counter = 1
        self.confirmed_victims = []  # Lista de dicts con la info de la víctima
        self.marker_array_msg = MarkerArray()

        # Timer para revivir los TFs en RViz constantemente
        self.timer = self.create_timer(1.0, self.timer_callback)

    def cloud_callback(self, msg: PointCloud2):
        try:
            points_generator = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            pts = np.array([[float(p[0]), float(p[1]), float(p[2])] for p in points_generator], dtype=np.float32)
            if len(pts) > 0:
                dists = np.linalg.norm(pts, axis=1)
                self.latest_cloud_points = pts[dists < LIDAR_MAX_RANGE]
        except Exception: 
            pass

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        # Republicar todos los TFs anclados en ODOM
        for v in self.confirmed_victims:
            self._publish_tf(v['id'], v['pos_odom'], now, 'odom')
            
        # Republicar marcadores visuales
        if self.marker_array_msg.markers:
            for m in self.marker_array_msg.markers:
                m.header.stamp = now
            self.pub_markers.publish(self.marker_array_msg)

    def detections_callback(self, msg: SpatialDetectionArray):
        # Corrección: Ya no bloqueamos si el LiDAR no está listo.
        if not msg.detections: 
            return

        ld_msg = LandmarkDetections()
        ld_msg.header.stamp = msg.header.stamp
        ld_msg.header.frame_id = 'unilidar_lidar'

        for det in msg.detections:
            if not det.results: continue
            best = max(det.results, key=lambda r: float(r.score))
            score_val = float(best.score)
            
            if score_val < MIN_CONFIDENCE: continue

            class_label = str(best.class_id)
            
            # Posición cruda de la cámara
            cam_x = float(det.position.x)
            cam_y = float(det.position.y)
            cam_z = float(det.position.z)
            
            cam_pos = np.array([cam_x, cam_y, cam_z], dtype=np.float64)
            norm = float(np.linalg.norm(cam_pos))
            if norm < 1e-6: continue

            # 1. Transformación de Ejes OAK-D a ROS Estándar (X=Z, Y=-X, Z=-Y)
            ros_x = cam_z
            ros_y = -cam_x
            ros_z = -cam_y

            # 2. Refinamiento con LiDAR
            ref_d = self._refine_depth(ros_x, ros_y, ros_z)
            scale = ref_d / np.linalg.norm([ros_x, ros_y, ros_z])
            pos_lidar = np.array([ros_x * scale, ros_y * scale, ros_z * scale], dtype=np.float64)

            # 3. Transformar a frame global (ODOM) para dedup
            pos_odom = self._transform_to_frame(pos_lidar, msg.header.frame_id, 'odom')
            if pos_odom is None: 
                # Si falla la transformación a odom, intentamos dedup en frame lidar como fallback
                pos_odom = pos_lidar

            # 4. Deduplicación Espacial
            is_new = True
            assigned_id = None
            
            for v in self.confirmed_victims:
                dist = float(np.linalg.norm(v['pos_odom'] - pos_odom))
                if dist < MIN_DIST_BETWEEN_VICTIMS:
                    is_new = False
                    assigned_id = v['id']
                    break
            
            if is_new:
                assigned_id = self.landmark_id_counter
                self.landmark_id_counter += 1
                
                self.confirmed_victims.append({
                    'id': assigned_id, 
                    'pos_odom': pos_odom, 
                    'pos_lidar': pos_lidar
                })
                
                self.get_logger().info(f'VÍCTIMA CONFIRMADA ID {assigned_id} a {ref_d:.2f}m. (C:{class_label})')
                self._add_visual_marker(assigned_id, pos_odom, class_label, score_val)

            # 5. Publicar Landmark para RTAB-Map
            lm = LandmarkDetection()
            lm.header.stamp = msg.header.stamp
            lm.header.frame_id = 'unilidar_lidar'
            lm.landmark_frame_id = f'victim_{assigned_id}'
            lm.id = int(assigned_id)
            lm.size = 0.3
            lm.pose.pose.position.x = float(pos_lidar[0])
            lm.pose.pose.position.y = float(pos_lidar[1])
            lm.pose.pose.position.z = float(pos_lidar[2])
            lm.pose.pose.orientation.w = 1.0
            cov = [0.0] * 36
            cov[0], cov[7], cov[14] = 0.05, 0.05, 0.10
            cov[21], cov[28], cov[35] = 0.01, 0.01, 0.01
            lm.pose.covariance = cov
            
            ld_msg.landmarks.append(lm)

        if ld_msg.landmarks:
            self.pub_landmarks.publish(ld_msg)

    def _add_visual_marker(self, lm_id, pos_global, class_label, score):
        color = CLASS_COLORS.get(class_label.lower(), CLASS_COLORS['default'])
        
        # Esfera
        m = Marker()
        m.header.frame_id = 'odom'
        m.ns = 'victims'
        m.id = int(lm_id)
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(pos_global[0])
        m.pose.position.y = float(pos_global[1])
        m.pose.position.z = float(pos_global[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.4
        m.color.r = float(color[0])
        m.color.g = float(color[1])
        m.color.b = float(color[2])
        m.color.a = 0.8
        m.lifetime.sec = 0 # Permanente
        self.marker_array_msg.markers.append(m)
        
        # Texto
        t = Marker()
        t.header.frame_id = 'odom'
        t.ns = 'labels'
        t.id = int(lm_id + 1000)
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = float(pos_global[0])
        t.pose.position.y = float(pos_global[1])
        t.pose.position.z = float(pos_global[2]) + 0.5
        t.pose.orientation.w = 1.0
        t.scale.z = 0.3
        t.color.r = t.color.g = t.color.b = t.color.a = 1.0
        t.text = f"VIC {lm_id}\nC:{class_label} S:{score:.0f}"
        t.lifetime.sec = 0 # Permanente
        self.marker_array_msg.markers.append(t)

    def _publish_tf(self, l_id, pos, stamp, parent_frame):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = str(parent_frame)
        t.child_frame_id = f'victim_{l_id}'
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _refine_depth(self, x, y, z):
        norm = float(np.linalg.norm([x, y, z]))
        if self.latest_cloud_points is None or len(self.latest_cloud_points) < 10: 
            return norm
            
        try:
            dir_vec = np.array([x, y, z], dtype=np.float64) / norm
            tf = self.tf_buffer.lookup_transform('unilidar_lidar', 'oak_rgb_camera_optical_frame', rclpy.time.Time())
            rot = quat_to_mat(tf.transform.rotation)
            dir_lidar = rot @ dir_vec
            
            pts = self.latest_cloud_points
            norms = np.linalg.norm(pts, axis=1, keepdims=True)
            norms[norms == 0] = 1e-6
            dots = np.dot(pts / norms, dir_lidar)
            pts_in_cone = pts[dots > np.cos(LIDAR_CONE_ANGLE_RAD)]
            
            if len(pts_in_cone) < 3: 
                return norm
                
            median_d = float(np.median(np.linalg.norm(pts_in_cone, axis=1)))
            if abs(median_d - norm) <= 2.0:
                return median_d
            return norm
        except Exception: 
            return norm

    def _transform_to_frame(self, pos, from_frame, to_frame):
        try:
            tf = self.tf_buffer.lookup_transform(str(to_frame), str(from_frame), rclpy.time.Time())
            rot = quat_to_mat(tf.transform.rotation)
            trans = np.array([float(tf.transform.translation.x), float(tf.transform.translation.y), float(tf.transform.translation.z)], dtype=np.float64)
            return rot @ pos + trans
        except Exception: 
            return None

def main(args=None):
    rclpy.init(args=args)
    node = LandmarkTFPublisher()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    except Exception as e:
        print(f"Node crashed: {e}")
    finally: 
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()