import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from tf2_ros import TransformBroadcaster


PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def ned_heading_to_enu_yaw(heading: float) -> float:
    return math.pi * 0.5 - heading


def px4_timestamp_to_stamp(timestamp_us: int):
    stamp_sec = int(timestamp_us // 1_000_000)
    stamp_nanosec = int((timestamp_us % 1_000_000) * 1000)
    return stamp_sec, stamp_nanosec


class Px4OdometryBridge(Node):
    def __init__(self):
        super().__init__('px4_odometry_bridge')

        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.local_position_topic = self.declare_parameter(
            'local_position_topic',
            '/fmu/out/vehicle_local_position_v1',
        ).value
        self.odom_topic = self.declare_parameter('odom_topic', '/px4/odom').value
        self.origin_x = float(self.declare_parameter('origin_x', 0.0).value)
        self.origin_y = float(self.declare_parameter('origin_y', -5.5).value)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            self.local_position_topic,
            self.local_position_cb,
            PX4_QOS,
        )
        self.get_logger().info(
            f'PX4 local position bridge started: {self.local_position_topic} -> {self.odom_topic}, '
            f'origin=({self.origin_x:.2f}, {self.origin_y:.2f})'
        )

    def local_position_cb(self, msg: VehicleLocalPosition) -> None:
        if math.isnan(msg.x) or math.isnan(msg.y) or math.isnan(msg.z):
            self.get_logger().warn(
                'Skipping PX4 local position: position contains NaN.',
                throttle_duration_sec=2.0,
            )
            return

        stamp = self.get_clock().now().to_msg()
        if stamp.sec == 0 and stamp.nanosec == 0 and getattr(msg, 'timestamp', 0) > 0:
            stamp.sec, stamp.nanosec = px4_timestamp_to_stamp(int(msg.timestamp))
        x = self.origin_x + float(msg.y)
        y = self.origin_y + float(msg.x)
        z = float(-msg.z)
        heading = 0.0 if math.isnan(msg.heading) else float(msg.heading)
        qx, qy, qz, qw = yaw_to_quaternion(ned_heading_to_enu_yaw(heading))

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.twist.twist.linear.x = float(msg.vy)
        odom.twist.twist.linear.y = float(msg.vx)
        odom.twist.twist.linear.z = float(-msg.vz)
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.w = qw
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = Px4OdometryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('PX4 odometry bridge stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
