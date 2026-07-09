import math
from typing import Iterable, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


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


class PointCloudToOccupancyGrid(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_to_occupancy_grid')

        self.cloud_topic = str(self.declare_parameter('cloud_topic', '/rtabmap/cloud_obstacles').value)
        self.map_topic = str(self.declare_parameter('map_topic', '/pointcloud_obstacle_map').value)
        self.target_frame = str(self.declare_parameter('target_frame', 'map').value)
        self.resolution = float(self.declare_parameter('resolution', 0.05).value)
        self.min_x = float(self.declare_parameter('min_x', -14.0).value)
        self.max_x = float(self.declare_parameter('max_x', 14.0).value)
        self.min_y = float(self.declare_parameter('min_y', -12.0).value)
        self.max_y = float(self.declare_parameter('max_y', 12.0).value)
        self.min_z = float(self.declare_parameter('min_z', 0.10).value)
        self.max_z = float(self.declare_parameter('max_z', 3.2).value)
        self.point_radius = float(self.declare_parameter('point_radius', 0.02).value)
        self.inflation_radius = float(self.declare_parameter('inflation_radius', 0.00).value)
        self.min_points_per_cell = int(self.declare_parameter('min_points_per_cell', 2).value)
        self.hit_count_threshold = int(self.declare_parameter('hit_count_threshold', 2).value)
        self.accumulate = parse_bool(self.declare_parameter('accumulate', True).value)
        self.publish_period = float(self.declare_parameter('publish_period', 1.0).value)

        self.width = int(math.ceil((self.max_x - self.min_x) / self.resolution))
        self.height = int(math.ceil((self.max_y - self.min_y) / self.resolution))
        self.occupancy = [0] * (self.width * self.height)
        self.hit_counts = [0] * (self.width * self.height)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self.cloud_sub = self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_cb, qos)
        self.timer = self.create_timer(self.publish_period, self.publish_map)
        self.last_cloud_stamp = None
        self.last_obstacle_count = 0

        self.get_logger().info(
            f'PointCloud obstacle map ready: {self.cloud_topic} -> {self.map_topic}, '
            f'frame={self.target_frame}, size={self.width}x{self.height}, '
            f'resolution={self.resolution:.2f}, z=[{self.min_z:.2f}, {self.max_z:.2f}], '
            f'point_radius={self.point_radius:.2f}, inflation={self.inflation_radius:.2f}, '
            f'min_points_per_cell={self.min_points_per_cell}, hit_count_threshold={self.hit_count_threshold}'
        )

    def cloud_cb(self, msg: PointCloud2) -> None:
        transform = self.lookup_transform(msg.header.frame_id)
        if transform is None:
            return

        if not self.accumulate:
            self.occupancy = [0] * (self.width * self.height)
            self.hit_counts = [0] * (self.width * self.height)

        cell_counts = {}
        for x, y, z in self.iter_points(msg, transform):
            if z < self.min_z or z > self.max_z:
                continue
            cell = self.world_to_grid(x, y)
            if cell is not None:
                cell_counts[cell] = cell_counts.get(cell, 0) + 1

        obstacle_cells = {
            cell for cell, count in cell_counts.items()
            if count >= self.min_points_per_cell
        }

        self.last_obstacle_count = len(obstacle_cells)
        self.last_cloud_stamp = msg.header.stamp
        self.apply_obstacles(obstacle_cells)

    def lookup_transform(self, source_frame: str) -> Optional[TransformStamped]:
        source_frame = source_frame or self.target_frame
        if source_frame == self.target_frame:
            transform = TransformStamped()
            transform.header.frame_id = self.target_frame
            transform.child_frame_id = source_frame
            transform.transform.rotation.w = 1.0
            return transform

        try:
            return self.tf_buffer.lookup_transform(
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

    def iter_points(
        self,
        msg: PointCloud2,
        transform: TransformStamped,
    ) -> Iterable[Tuple[float, float, float]]:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        identity = (
            transform.header.frame_id == self.target_frame
            and transform.child_frame_id == self.target_frame
        )

        for point in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            if identity:
                yield x, y, z
                continue

            rotated = rotate_vector(rotation, np.array([x, y, z], dtype=float))
            yield (
                float(rotated[0] + translation.x),
                float(rotated[1] + translation.y),
                float(rotated[2] + translation.z),
            )

    def apply_obstacles(self, obstacle_cells) -> None:
        mark_radius = max(self.point_radius, self.inflation_radius)
        inflation_cells = max(0, int(math.ceil(mark_radius / self.resolution)))
        for cx, cy in obstacle_cells:
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    if dx * dx + dy * dy > inflation_cells * inflation_cells:
                        continue
                    x = cx + dx
                    y = cy + dy
                    if 0 <= x < self.width and 0 <= y < self.height:
                        index = y * self.width + x
                        self.hit_counts[index] += 1
                        if self.hit_counts[index] >= self.hit_count_threshold:
                            self.occupancy[index] = 100

    def world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        gx = int((x - self.min_x) / self.resolution)
        gy = int((y - self.min_y) / self.resolution)
        if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
            return None
        return gx, gy

    def publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.min_x
        msg.info.origin.position.y = self.min_y
        msg.info.origin.orientation.w = 1.0
        msg.data = list(self.occupancy)
        self.map_pub.publish(msg)
        self.get_logger().info(
            f'Published {self.map_topic}: occupied_cells={self.occupancy.count(100)}, '
            f'last_cloud_obstacles={self.last_obstacle_count}',
            throttle_duration_sec=3.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudToOccupancyGrid()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
