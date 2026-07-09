import math
import os
from collections import deque
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from visualization_msgs.msg import Marker


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def rotate_vector(quaternion, vector: np.ndarray) -> np.ndarray:
    qx = quaternion.x
    qy = quaternion.y
    qz = quaternion.z
    qw = quaternion.w

    q_vec = np.array([qx, qy, qz], dtype=float)
    uv = np.cross(q_vec, vector)
    uuv = np.cross(q_vec, uv)
    return vector + 2.0 * (qw * uv + uuv)


class ArucoGoalDetector(Node):
    def __init__(self) -> None:
        super().__init__('aruco_goal_detector')

        self.image_topic = str(self.declare_parameter('image_topic', '/rgb_camera').value)
        self.camera_info_topic = str(
            self.declare_parameter('camera_info_topic', '/rgb_camera/camera_info').value
        )
        self.goal_topic = str(self.declare_parameter('goal_topic', '/aruco/goal_pose').value)
        self.marker_topic = str(self.declare_parameter('marker_topic', '/aruco/marker').value)
        self.debug_image_topic = str(
            self.declare_parameter('debug_image_topic', '/aruco/debug_image').value
        )
        self.dictionary_name = str(self.declare_parameter('dictionary', 'DICT_5X5_100').value)
        self.marker_id = int(self.declare_parameter('marker_id', 23).value)
        self.target_frame = str(self.declare_parameter('target_frame', 'odom').value)
        self.ground_z = float(self.declare_parameter('ground_z', 0.0).value)
        self.horizontal_fov = float(self.declare_parameter('horizontal_fov', 1.204).value)
        self.publish_debug_image = parse_bool(
            self.declare_parameter('publish_debug_image', True).value
        )
        self.save_pose = parse_bool(self.declare_parameter('save_pose', True).value)
        self.save_pose_path = os.path.expanduser(
            str(self.declare_parameter('save_pose_path', '~/main_ws/maps/aruco_marker.yaml').value)
        )
        self.filter_window_size = int(self.declare_parameter('filter_window_size', 30).value)
        self.min_stable_samples = int(self.declare_parameter('min_stable_samples', 10).value)
        self.max_position_std = float(self.declare_parameter('max_position_std', 0.35).value)

        dictionary_id = getattr(cv2.aruco, self.dictionary_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.detector = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, qos)
        self.create_subscription(Image, self.image_topic, self.image_cb, qos)
        self.last_saved_pose: Optional[Tuple[float, float]] = None
        self.position_samples = deque(maxlen=max(1, self.filter_window_size))

        self.get_logger().info(
            f'Aruco detector ready: image={self.image_topic}, marker_id={self.marker_id}, '
            f'target_frame={self.target_frame}, goal_topic={self.goal_topic}, '
            f'save_pose_path={self.save_pose_path}, filter_window={self.filter_window_size}, '
            f'min_samples={self.min_stable_samples}'
        )

    def camera_info_cb(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def image_cb(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert RGB image: {exc}', throttle_duration_sec=2.0)
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect_markers(gray)
        if ids is None or len(ids) == 0:
            self.publish_debug(msg, image, corners, ids)
            return

        flat_ids = ids.flatten()
        matches = np.where(flat_ids == self.marker_id)[0]
        if len(matches) == 0:
            self.publish_debug(msg, image, corners, ids)
            return

        marker_index = int(matches[0])
        center_u, center_v = self.marker_center(corners[marker_index])
        goal = self.project_pixel_to_ground(msg, center_u, center_v, image.shape[1], image.shape[0])
        if goal is None:
            self.publish_debug(msg, image, corners, ids)
            return

        filtered = self.update_filtered_pose(goal[0], goal[1])
        publish_x, publish_y = filtered if filtered is not None else goal

        pose = self.make_pose(publish_x, publish_y)
        self.goal_pub.publish(pose)
        self.marker_pub.publish(self.make_marker(pose))
        if filtered is not None:
            self.save_marker_pose(filtered[0], filtered[1])

        if filtered is None:
            self.get_logger().info(
                f'ArUco {self.marker_id} detected raw '
                f'{self.target_frame}=({goal[0]:.2f}, {goal[1]:.2f}, {self.ground_z:.2f}); '
                f'collecting stable samples {len(self.position_samples)}/{self.min_stable_samples}',
                throttle_duration_sec=1.0,
            )
        else:
            self.get_logger().info(
                f'ArUco {self.marker_id} stable pose '
                f'{self.target_frame}=({filtered[0]:.2f}, {filtered[1]:.2f}, {self.ground_z:.2f})',
                throttle_duration_sec=1.0,
            )
        self.publish_debug(msg, image, corners, ids)

    def make_pose(self, x: float, y: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.target_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = self.ground_z
        pose.pose.orientation.w = 1.0
        return pose

    def detect_markers(self, gray):
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
            return corners, ids
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.detector_params,
        )
        return corners, ids

    def marker_center(self, marker_corners) -> Tuple[float, float]:
        points = marker_corners.reshape((4, 2))
        center = points.mean(axis=0)
        return float(center[0]), float(center[1])

    def project_pixel_to_ground(
        self,
        msg: Image,
        u: float,
        v: float,
        width: int,
        height: int,
    ) -> Optional[Tuple[float, float]]:
        intrinsics = self.get_intrinsics(width, height)
        if intrinsics is None:
            return None
        fx, fy, cx, cy = intrinsics

        source_frame = msg.header.frame_id or 'camera_rgb_optical_frame'
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Waiting for TF {self.target_frame} <- {source_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        ray_camera = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=float)
        ray_camera /= np.linalg.norm(ray_camera)
        ray_target = rotate_vector(transform.transform.rotation, ray_camera)
        ray_target /= np.linalg.norm(ray_target)

        origin = transform.transform.translation
        if abs(ray_target[2]) < 1e-6:
            self.get_logger().warn('Camera ray is nearly parallel to the ground.', throttle_duration_sec=2.0)
            return None

        distance = (self.ground_z - origin.z) / ray_target[2]
        if distance <= 0.0:
            self.get_logger().warn('Projected ArUco ray points away from the ground.', throttle_duration_sec=2.0)
            return None

        x = origin.x + distance * ray_target[0]
        y = origin.y + distance * ray_target[1]
        return x, y

    def update_filtered_pose(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        self.position_samples.append((x, y))
        if len(self.position_samples) < self.min_stable_samples:
            return None

        samples = np.array(self.position_samples, dtype=float)
        median = np.median(samples, axis=0)
        distances = np.linalg.norm(samples - median, axis=1)
        std = float(np.std(distances))
        if std > self.max_position_std:
            self.get_logger().warn(
                f'ArUco pose samples are not stable yet: std={std:.2f} m, '
                f'limit={self.max_position_std:.2f} m',
                throttle_duration_sec=2.0,
            )
            return None

        return float(median[0]), float(median[1])

    def save_marker_pose(self, x: float, y: float) -> None:
        if not self.save_pose:
            return

        if self.last_saved_pose is not None:
            if math.hypot(x - self.last_saved_pose[0], y - self.last_saved_pose[1]) < 0.02:
                return

        directory = os.path.dirname(self.save_pose_path)
        if directory:
            os.makedirs(directory, exist_ok=True)

        content = (
            f'marker_id: {self.marker_id}\n'
            f'frame_id: {self.target_frame}\n'
            f'x: {x:.6f}\n'
            f'y: {y:.6f}\n'
            f'z: {self.ground_z:.6f}\n'
            f'samples: {len(self.position_samples)}\n'
        )
        with open(self.save_pose_path, 'w', encoding='utf-8') as stream:
            stream.write(content)
        self.last_saved_pose = (x, y)
        self.get_logger().info(f'Saved ArUco pose to {self.save_pose_path}', throttle_duration_sec=2.0)

    def get_intrinsics(self, width: int, height: int) -> Optional[Tuple[float, float, float, float]]:
        if self.camera_info is not None and self.camera_info.k[0] > 0.0 and self.camera_info.k[4] > 0.0:
            return (
                float(self.camera_info.k[0]),
                float(self.camera_info.k[4]),
                float(self.camera_info.k[2]),
                float(self.camera_info.k[5]),
            )

        if width <= 0 or height <= 0:
            return None

        fx = width / (2.0 * math.tan(self.horizontal_fov * 0.5))
        fy = fx
        cx = width * 0.5
        cy = height * 0.5
        return fx, fy, cx, cy

    def make_marker(self, pose: PoseStamped) -> Marker:
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'aruco_goal'
        marker.id = self.marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = self.ground_z + 0.05
        marker.scale.x = 0.45
        marker.scale.y = 0.45
        marker.scale.z = 0.10
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.4
        marker.color.b = 1.0
        return marker

    def publish_debug(self, msg: Image, image, corners, ids) -> None:
        if not self.publish_debug_image:
            return
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
        debug_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoGoalDetector()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
