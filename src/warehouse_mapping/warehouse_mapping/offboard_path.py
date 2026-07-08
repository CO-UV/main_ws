import math
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


@dataclass(frozen=True)
class Waypoint:
    north: float
    east: float
    down: float
    yaw: float = 0.0
    hold_s: float = 0.0


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def gazebo_xyz_to_px4_ned(
    x_east: float,
    y_north: float,
    z_up: float,
    yaw_enu: float = 0.0,
    hold_s: float = 0.0,
) -> Waypoint:
    # Gazebo world is ENU-like: x=east, y=north, z=up.
    # PX4 trajectory setpoints use NED: x=north, y=east, z=down.
    return Waypoint(
        north=y_north,
        east=x_east,
        down=-z_up,
        yaw=math.pi / 2.0 - yaw_enu,
        hold_s=hold_s,
    )


class WarehouseOffboardPath(Node):
    def __init__(self) -> None:
        super().__init__('warehouse_offboard_path')

        self.altitude = float(self.declare_parameter('altitude', 3.5).value)
        self.acceptance_radius = float(self.declare_parameter('acceptance_radius', 0.80).value)
        self.hold_time_s = float(self.declare_parameter('hold_time_s', 0.0).value)
        self.auto_arm = parse_bool(self.declare_parameter('auto_arm', True).value)
        self.auto_land = parse_bool(self.declare_parameter('auto_land', True).value)
        self.start_delay_s = float(self.declare_parameter('start_delay_s', 3.0).value)
        self.max_speed = float(self.declare_parameter('max_speed', 0.15).value)
        self.require_status = parse_bool(self.declare_parameter('require_status', False).value)
        self.status_topic = str(self.declare_parameter('status_topic', '/fmu/out/vehicle_status').value)
        self.local_position_topic = str(
            self.declare_parameter('local_position_topic', '/fmu/out/vehicle_local_position_v1').value
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10,
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10,
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            self.status_topic,
            self.vehicle_status_cb,
            PX4_QOS,
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            self.local_position_topic,
            self.local_position_cb,
            PX4_QOS,
        )

        self.status: Optional[VehicleStatus] = None
        self.local_position: Optional[VehicleLocalPosition] = None
        self.current_index = 0
        self.hold_started_ns: Optional[int] = None
        self.offboard_requested = False
        self.arm_requested = False
        self.speed_requested = False
        self.land_requested = False
        self.finished = False
        self.last_offboard_request_ns = 0
        self.last_arm_request_ns = 0
        self.last_speed_request_ns = 0
        self.start_time_ns = self.get_clock().now().nanoseconds

        self.waypoints = self.create_warehouse_path(self.altitude)

        self.timer = self.create_timer(0.05, self.timer_cb)
        self.get_logger().info(
            f'Warehouse offboard path ready: {len(self.waypoints)} waypoints, '
            f'altitude={self.altitude:.1f} m, auto_arm={self.auto_arm}, '
            f'local_position_topic={self.local_position_topic}'
        )

    def create_warehouse_path(self, z_up: float) -> List[Waypoint]:
        # Very conservative Gazebo-world route for the Fuel warehouse model.
        #
        # Known obstacle locations in tugbot_warehouse.sdf:
        # - shelf rows: x=-4.4 and x=5.6, y=-0.7..8.4
        # - rear big shelves: centered at y=-13.0 with 18 m collision length,
        #   so their collision band reaches approximately y=-22..-4
        # - front pallets/carts: y=12.2..15.3, x=-6.1..14.0
        # - right rear shelves / tugbot: x=13.4..14.7, y=-24.3..-10.6
        #
        # Open-top UGV warehouse scan route. The custom world is deliberately
        # built for occupancy-map evaluation: no ceiling beams, box-shaped
        # obstacles, and broad free corridors for safe drone overflight.
        gazebo_points = [
            # Start directly above the spawn point, then translate gradually.
            # Jumping to the first scan line immediately after arming can make
            # PX4 lean hard enough to trigger attitude preflight/failsafe checks.
            (0.0, 0.0, z_up, 0.0, 1.0),
            (0.0, -3.0, z_up, math.pi),
            (0.0, -6.0, z_up, math.pi),
            (0.0, -9.0, z_up, math.pi),
            (-8.0, -9.0, z_up, 0.0),
            (-8.0, -6.0, z_up, 0.0),
            (-8.0, -3.0, z_up, 0.0),
            (-8.0, 0.0, z_up, 0.0),
            (-8.0, 3.0, z_up, 0.0),
            (-8.0, 6.0, z_up, 0.0),
            (-8.0, 9.0, z_up, 0.0),

            (-4.0, 9.0, z_up, math.pi),
            (-4.0, 6.0, z_up, math.pi),
            (-4.0, 3.0, z_up, math.pi),
            (-4.0, 0.0, z_up, math.pi),
            (-4.0, -3.0, z_up, math.pi),
            (-4.0, -6.0, z_up, math.pi),
            (-4.0, -9.0, z_up, math.pi),

            (0.0, -9.0, z_up, 0.0),
            (0.0, -6.0, z_up, 0.0),
            (0.0, -3.0, z_up, 0.0),
            (0.0, 0.0, z_up, 0.0),
            (0.0, 3.0, z_up, 0.0),
            (0.0, 6.0, z_up, 0.0),
            (0.0, 9.0, z_up, 0.0),

            (4.0, 9.0, z_up, math.pi),
            (4.0, 6.0, z_up, math.pi),
            (4.0, 3.0, z_up, math.pi),
            (4.0, 0.0, z_up, math.pi),
            (4.0, -3.0, z_up, math.pi),
            (4.0, -6.0, z_up, math.pi),
            (4.0, -9.0, z_up, math.pi),

            (8.0, -9.0, z_up, 0.0),
            (8.0, -6.0, z_up, 0.0),
            (8.0, -3.0, z_up, 0.0),
            (8.0, 0.0, z_up, 0.0),
            (8.0, 3.0, z_up, 0.0),
            (8.0, 6.0, z_up, 0.0),
            (8.0, 9.0, z_up, 0.0),

            (0.0, 9.0, z_up, math.pi),
            (0.0, 6.0, z_up, math.pi),
            (0.0, 3.0, z_up, math.pi),
            (0.0, 0.0, z_up, math.pi),
            (0.0, -3.0, z_up, math.pi),
            (0.0, -6.0, z_up, math.pi),
            (0.0, -9.0, z_up, math.pi, 1.0),
        ]

        coarse_path = [
            gazebo_xyz_to_px4_ned(*point)
            for point in gazebo_points
        ]
        return self.interpolate_path(coarse_path, spacing_m=0.75)

    def interpolate_path(self, path: List[Waypoint], spacing_m: float) -> List[Waypoint]:
        if len(path) < 2:
            return path

        dense = [path[0]]
        for start, end in zip(path, path[1:]):
            dn = end.north - start.north
            de = end.east - start.east
            dd = end.down - start.down
            distance = math.sqrt(dn * dn + de * de + dd * dd)
            steps = max(1, int(math.ceil(distance / spacing_m)))
            for step in range(1, steps + 1):
                ratio = step / steps
                hold_s = end.hold_s if step == steps else 0.0
                dense.append(
                    Waypoint(
                        north=start.north + dn * ratio,
                        east=start.east + de * ratio,
                        down=start.down + dd * ratio,
                        yaw=end.yaw,
                        hold_s=hold_s,
                    )
                )
        return dense

    def vehicle_status_cb(self, msg: VehicleStatus) -> None:
        self.status = msg

    def local_position_cb(self, msg: VehicleLocalPosition) -> None:
        self.local_position = msg

    def timer_cb(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds

        self.publish_offboard_control_mode()

        if self.local_position is None or not self.local_position.xy_valid or not self.local_position.z_valid:
            self.publish_setpoint(self.waypoints[0])
            self.get_logger().warn('Waiting for valid PX4 local position...', throttle_duration_sec=2.0)
            return

        if (now_ns - self.start_time_ns) / 1e9 < self.start_delay_s:
            self.publish_setpoint(self.waypoints[0])
            return

        if (not self.require_status or not self.is_offboard()) and self.should_retry(
            now_ns,
            self.last_offboard_request_ns,
        ):
            self.set_offboard_mode()
            self.last_offboard_request_ns = now_ns
            self.offboard_requested = True

        if not self.speed_requested and self.should_retry(now_ns, self.last_speed_request_ns):
            self.set_speed(self.max_speed)
            self.last_speed_request_ns = now_ns
            self.speed_requested = True

        if self.auto_arm and (not self.require_status or not self.is_armed()) and self.should_retry(
            now_ns,
            self.last_arm_request_ns,
        ):
            self.arm()
            self.last_arm_request_ns = now_ns
            self.arm_requested = True

        if self.require_status and (not self.is_offboard() or (self.auto_arm and not self.is_armed())):
            self.publish_setpoint(self.waypoints[0])
            self.get_logger().warn(
                f'Waiting for PX4 offboard/arm: offboard={self.is_offboard()} armed={self.is_armed()}',
                throttle_duration_sec=2.0,
            )
            return

        if self.finished:
            self.publish_setpoint(self.waypoints[-1])
            return

        waypoint = self.waypoints[self.current_index]
        self.publish_setpoint(waypoint)

        if self.reached(waypoint):
            if waypoint.hold_s <= 0.0:
                self.advance_waypoint()
                return

            if self.hold_started_ns is None:
                self.hold_started_ns = now_ns
                self.get_logger().info(
                    f'Waypoint {self.current_index + 1}/{len(self.waypoints)} reached '
                    f'(north={waypoint.north:.1f}, east={waypoint.east:.1f}, down={waypoint.down:.1f}).'
                )
                return

            hold_elapsed = (now_ns - self.hold_started_ns) / 1e9
            hold_required = max(waypoint.hold_s, self.hold_time_s)
            if hold_elapsed < hold_required:
                return

            self.advance_waypoint()

    def advance_waypoint(self) -> None:
        self.current_index += 1
        self.hold_started_ns = None
        if self.current_index >= len(self.waypoints):
            self.finished = True
            self.get_logger().info('Warehouse offboard path complete.')
            if self.auto_land and not self.land_requested:
                self.land()
                self.land_requested = True

    def reached(self, waypoint: Waypoint) -> bool:
        if self.local_position is None:
            return False
        dx = float(self.local_position.x) - waypoint.north
        dy = float(self.local_position.y) - waypoint.east
        dz = float(self.local_position.z) - waypoint.down
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        return distance <= self.acceptance_radius

    def should_retry(self, now_ns: int, last_request_ns: int) -> bool:
        return last_request_ns == 0 or (now_ns - last_request_ns) / 1e9 >= 1.0

    def is_offboard(self) -> bool:
        return (
            self.status is not None
            and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def is_armed(self) -> bool:
        return (
            self.status is not None
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_control_mode_pub.publish(msg)

    def publish_setpoint(self, waypoint: Waypoint) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us()
        msg.position = [float(waypoint.north), float(waypoint.east), float(waypoint.down)]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = float(waypoint.yaw)
        msg.yawspeed = math.nan
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.command = command
        msg.param1 = float(params.get('param1', 0.0))
        msg.param2 = float(params.get('param2', 0.0))
        msg.param3 = float(params.get('param3', 0.0))
        msg.param4 = float(params.get('param4', 0.0))
        msg.param5 = float(params.get('param5', 0.0))
        msg.param6 = float(params.get('param6', 0.0))
        msg.param7 = float(params.get('param7', 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def set_offboard_mode(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.get_logger().info('Requested PX4 offboard mode.')

    def arm(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.get_logger().info('Requested vehicle arm.')

    def set_speed(self, speed_m_s: float) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_CHANGE_SPEED,
            param1=float(VehicleCommand.SPEED_TYPE_GROUNDSPEED),
            param2=float(speed_m_s),
            param3=-1.0,
        )
        self.get_logger().info(f'Requested ground speed limit {speed_m_s:.2f} m/s.')

    def land(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Requested vehicle land.')

    def timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WarehouseOffboardPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Warehouse offboard path stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
