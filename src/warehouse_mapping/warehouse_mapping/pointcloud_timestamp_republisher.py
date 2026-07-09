import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class PointCloudTimestampRepublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_timestamp_republisher')

        self.input_topic = self.declare_parameter(
            'input_topic',
            '/depth_camera/points',
        ).value
        self.output_topic = self.declare_parameter(
            'output_topic',
            '/depth_camera/points_synced',
        ).value
        self.frame_id = self.declare_parameter('frame_id', '').value
        self.enable_altitude_gate = bool(
            self.declare_parameter('enable_altitude_gate', False).value
        )
        self.local_position_topic = self.declare_parameter(
            'local_position_topic',
            '/fmu/out/vehicle_local_position_v1',
        ).value
        self.target_altitude = float(self.declare_parameter('target_altitude', 4.0).value)
        self.altitude_tolerance = float(self.declare_parameter('altitude_tolerance', 0.35).value)
        self.stable_time_s = float(self.declare_parameter('stable_time_s', 1.5).value)
        self.enable_mapping_active_gate = bool(
            self.declare_parameter('enable_mapping_active_gate', False).value
        )
        self.mapping_active_topic = self.declare_parameter(
            'mapping_active_topic',
            '/mapping/active',
        ).value

        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_cb,
            qos_profile_sensor_data,
        )
        self.local_position_subscription = self.create_subscription(
            VehicleLocalPosition,
            self.local_position_topic,
            self.local_position_cb,
            qos_profile_sensor_data,
        )
        self.mapping_active_subscription = self.create_subscription(
            Bool,
            self.mapping_active_topic,
            self.mapping_active_cb,
            10,
        )
        self.altitude_gate_started_ns = None
        self.altitude_gate_open = not self.enable_altitude_gate
        self.current_altitude = None
        self.mapping_active = not self.enable_mapping_active_gate
        self.get_logger().info(
            f'Republishing point cloud timestamps: {self.input_topic} -> {self.output_topic}'
        )
        if self.enable_altitude_gate:
            self.get_logger().info(
                f'Point cloud altitude gate enabled: target={self.target_altitude:.2f} m, '
                f'tolerance={self.altitude_tolerance:.2f} m, stable_time={self.stable_time_s:.1f} s, '
                f'local_position_topic={self.local_position_topic}'
            )
        if self.enable_mapping_active_gate:
            self.get_logger().info(
                f'Point cloud mapping gate enabled: mapping_active_topic={self.mapping_active_topic}'
            )

    def mapping_active_cb(self, msg: Bool) -> None:
        self.mapping_active = bool(msg.data)

    def local_position_cb(self, msg: VehicleLocalPosition) -> None:
        if not msg.z_valid:
            self.altitude_gate_started_ns = None
            self.current_altitude = None
            return

        self.current_altitude = -float(msg.z)
        altitude_error = abs(self.current_altitude - self.target_altitude)
        if altitude_error > self.altitude_tolerance:
            self.altitude_gate_started_ns = None
            if self.altitude_gate_open:
                self.get_logger().warn(
                    f'Point cloud altitude gate closed: altitude={self.current_altitude:.2f} m, '
                    f'target={self.target_altitude:.2f} m',
                    throttle_duration_sec=2.0,
                )
            self.altitude_gate_open = False
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.altitude_gate_started_ns is None:
            self.altitude_gate_started_ns = now_ns
            return

        stable_elapsed = (now_ns - self.altitude_gate_started_ns) / 1e9
        if stable_elapsed >= self.stable_time_s and not self.altitude_gate_open:
            self.altitude_gate_open = True
            self.get_logger().info(
                f'Point cloud altitude gate opened at altitude={self.current_altitude:.2f} m.'
            )

    def pointcloud_cb(self, msg: PointCloud2) -> None:
        if not self.mapping_active:
            self.get_logger().warn(
                f'Dropping point cloud while mapping gate is inactive: {self.mapping_active_topic}=false',
                throttle_duration_sec=2.0,
            )
            return

        if not self.altitude_gate_open:
            altitude_text = 'unknown' if self.current_altitude is None else f'{self.current_altitude:.2f} m'
            self.get_logger().warn(
                f'Dropping point cloud until target altitude is stable: current_altitude={altitude_text}, '
                f'target={self.target_altitude:.2f} m',
                throttle_duration_sec=2.0,
            )
            return

        stamped_msg = copy.copy(msg)
        stamped_msg.header = copy.copy(msg.header)
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        if self.frame_id:
            stamped_msg.header.frame_id = self.frame_id
        self.publisher.publish(stamped_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTimestampRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Point cloud timestamp republisher stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
