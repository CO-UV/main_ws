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
from std_msgs.msg import Bool


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
    mapping_active: bool = True


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
    mapping_active: bool = True,
    origin_x_east: float = 0.0,
    origin_y_north: float = 0.0,
) -> Waypoint:
    # Gazebo world is ENU-like: x=east, y=north, z=up.
    # PX4 local setpoints use NED relative to the vehicle spawn/home origin:
    # x=north, y=east, z=down.
    return Waypoint(
        north=y_north - origin_y_north,
        east=x_east - origin_x_east,
        down=-z_up,
        yaw=math.pi / 2.0 - yaw_enu,
        hold_s=hold_s,
        mapping_active=mapping_active,
    )


class WarehouseOffboardPath(Node):
    def __init__(self) -> None:
        super().__init__('warehouse_offboard_path')

        self.altitude = float(self.declare_parameter('altitude', 4.2).value)
        self.acceptance_radius = float(self.declare_parameter('acceptance_radius', 0.70).value)
        self.land_radius = float(self.declare_parameter('land_radius', 1.50).value)
        self.hold_time_s = float(self.declare_parameter('hold_time_s', 0.10).value)
        self.auto_arm = parse_bool(self.declare_parameter('auto_arm', True).value)
        self.auto_land = parse_bool(self.declare_parameter('auto_land', True).value)
        self.start_delay_s = float(self.declare_parameter('start_delay_s', 3.0).value)
        self.max_speed = float(self.declare_parameter('max_speed', 0.04).value)
        self.spawn_x = float(self.declare_parameter('spawn_x', 0.0).value)
        self.spawn_y = float(self.declare_parameter('spawn_y', -5.5).value)
        self.require_status = parse_bool(self.declare_parameter('require_status', True).value)
        self.status_topic = str(self.declare_parameter('status_topic', '/fmu/out/vehicle_status_v1').value)
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
        self.mapping_active_pub = self.create_publisher(Bool, '/mapping/active', 10)
        self.vehicle_status_subscriptions = []
        self.subscribe_vehicle_status_topics()
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
        self.last_land_request_ns = 0
        self.last_progress_log_ns = 0
        self.start_time_ns = self.get_clock().now().nanoseconds

        self.waypoints = self.create_warehouse_path(self.altitude)

        self.timer = self.create_timer(0.05, self.timer_cb)
        self.get_logger().info(
            f'Warehouse offboard path ready: {len(self.waypoints)} waypoints, '
            f'altitude={self.altitude:.1f} m, auto_arm={self.auto_arm}, '
            f'acceptance_radius={self.acceptance_radius:.2f} m, '
            f'hold_time={self.hold_time_s:.2f} s, land_radius={self.land_radius:.1f} m, '
            f'spawn=({self.spawn_x:.1f}, {self.spawn_y:.1f}), '
            f'status_topic={self.status_topic}, local_position_topic={self.local_position_topic}'
        )

    def subscribe_vehicle_status_topics(self) -> None:
        status_topics = [self.status_topic]
        for fallback_topic in ('/fmu/out/vehicle_status_v1', '/fmu/out/vehicle_status'):
            if fallback_topic not in status_topics:
                status_topics.append(fallback_topic)

        for topic in status_topics:
            self.vehicle_status_subscriptions.append(
                self.create_subscription(
                    VehicleStatus,
                    topic,
                    self.vehicle_status_cb,
                    PX4_QOS,
                )
            )
        self.get_logger().info(f'Subscribed PX4 vehicle status topics: {status_topics}')

    def create_warehouse_path(self, z_up: float) -> List[Waypoint]:
        spawn_x = self.spawn_x
        spawn_y = self.spawn_y
        climb_hold_s = 3.0
        intermediate_z = min(2.5, z_up)

        # This is the wider lawnmower scan that produced the cleanest RTAB
        # grid in this warehouse: full-height vertical lanes, no extra cross
        # sweep, and only a final return to the spawn point.
        min_x = -11.0
        max_x = 11.0
        min_y = -10.5
        max_y = 10.5
        lane_xs = [-11.0, -8.5, -6.0, -3.5, -1.0, 1.5, 4.0, 6.5, 9.0, 11.0]

        gazebo_points = [
            # Spawn launch defaults are x=0.0, y=-5.5, z=1.2. Climb straight
            # up and hover before any lateral movement, then enter the scan
            # rectangle gradually.
            (spawn_x, spawn_y, intermediate_z, -math.pi / 2.0),
            (spawn_x, spawn_y, z_up, -math.pi / 2.0, climb_hold_s),
            (0.0, min_y, z_up, -math.pi / 2.0),
            (min_x, min_y, z_up, math.pi),
        ]

        for index, x in enumerate(lane_xs):
            if index == 0:
                start_y = min_y
                end_y = max_y
            elif index % 2 == 1:
                start_y = max_y
                end_y = min_y
                gazebo_points.append((x, start_y, z_up, 0.0))
            else:
                start_y = min_y
                end_y = max_y
                gazebo_points.append((x, start_y, z_up, 0.0))

            yaw = math.pi / 2.0 if end_y > start_y else -math.pi / 2.0
            gazebo_points.append((x, end_y, z_up, yaw))

        gazebo_points.extend([
            # Do not update RTAB during the return-to-home leg. This prevents
            # duplicated obstacles from small pose drift on the way back.
            (max_x, spawn_y, z_up, -math.pi / 2.0, 0.0, False),
            (spawn_x, spawn_y, z_up, math.pi, 1.0, False),
        ])

        coarse_path = [
            gazebo_xyz_to_px4_ned(
                *point,
                origin_x_east=spawn_x,
                origin_y_north=spawn_y,
            )
            for point in gazebo_points
        ]
        return self.interpolate_path(coarse_path, spacing_m=0.35)

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
                        mapping_active=end.mapping_active,
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

        if self.finished:
            self.publish_mapping_active(False)
            self.handle_finished(now_ns)
            return

        self.publish_offboard_control_mode()

        if self.local_position is None or not self.local_position.xy_valid or not self.local_position.z_valid:
            self.publish_mapping_active(False)
            self.publish_setpoint(self.waypoints[0])
            self.get_logger().warn('Waiting for valid PX4 local position...', throttle_duration_sec=2.0)
            return

        if (now_ns - self.start_time_ns) / 1e9 < self.start_delay_s:
            self.publish_mapping_active(False)
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

        if self.require_status and self.status is None:
            self.publish_mapping_active(False)
            self.publish_setpoint(self.waypoints[0])
            self.get_logger().warn(
                f'Waiting for PX4 vehicle status on {self.status_topic} '
                'or /fmu/out/vehicle_status_v1...',
                throttle_duration_sec=2.0,
            )
            return

        if self.require_status and (not self.is_offboard() or (self.auto_arm and not self.is_armed())):
            self.publish_mapping_active(False)
            self.publish_setpoint(self.waypoints[0])
            self.get_logger().warn(
                f'Waiting for PX4 offboard/arm: offboard={self.is_offboard()} armed={self.is_armed()} '
                f'nav_state={self.nav_state_text()} arming_state={self.arming_state_text()}',
                throttle_duration_sec=2.0,
            )
            return

        waypoint = self.waypoints[self.current_index]
        self.publish_mapping_active(waypoint.mapping_active)
        self.publish_setpoint(waypoint)
        self.log_progress(now_ns, waypoint)

        if self.is_final_waypoint() and self.horizontal_distance_to(waypoint) <= self.land_radius:
            self.finished = True
            self.get_logger().info(
                f'Return point reached within landing radius '
                f'({self.horizontal_distance_to(waypoint):.2f} <= {self.land_radius:.2f} m).'
            )
            if self.auto_land and not self.land_requested:
                self.land()
                self.land_requested = True
                self.last_land_request_ns = now_ns
            return

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
                self.last_land_request_ns = self.get_clock().now().nanoseconds
        else:
            waypoint = self.waypoints[self.current_index]
            self.get_logger().info(
                f'Advancing to waypoint {self.current_index + 1}/{len(self.waypoints)} '
                f'(north={waypoint.north:.1f}, east={waypoint.east:.1f}, down={waypoint.down:.1f}).'
            )

    def handle_finished(self, now_ns: int) -> None:
        if not self.auto_land:
            self.publish_setpoint(self.waypoints[-1])
            return

        if not self.is_armed():
            self.get_logger().info('Vehicle is no longer armed; landing sequence complete.', throttle_duration_sec=5.0)
            return

        if self.should_retry(now_ns, self.last_land_request_ns):
            self.land()
            self.last_land_request_ns = now_ns

    def reached(self, waypoint: Waypoint) -> bool:
        return self.distance_to(waypoint) <= self.acceptance_radius

    def is_final_waypoint(self) -> bool:
        return self.current_index >= len(self.waypoints) - 1

    def distance_to(self, waypoint: Waypoint) -> float:
        if self.local_position is None:
            return math.inf
        dx = float(self.local_position.x) - waypoint.north
        dy = float(self.local_position.y) - waypoint.east
        dz = float(self.local_position.z) - waypoint.down
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def horizontal_distance_to(self, waypoint: Waypoint) -> float:
        if self.local_position is None:
            return math.inf
        dx = float(self.local_position.x) - waypoint.north
        dy = float(self.local_position.y) - waypoint.east
        return math.sqrt(dx * dx + dy * dy)

    def log_progress(self, now_ns: int, waypoint: Waypoint) -> None:
        if self.local_position is None:
            return
        if self.last_progress_log_ns != 0 and (now_ns - self.last_progress_log_ns) / 1e9 < 2.0:
            return
        self.last_progress_log_ns = now_ns
        self.get_logger().info(
            f'Waypoint {self.current_index + 1}/{len(self.waypoints)}: '
            f'distance={self.distance_to(waypoint):.2f} m, '
            f'current=({float(self.local_position.x):.1f}, {float(self.local_position.y):.1f}, '
            f'{float(self.local_position.z):.1f}), '
            f'target=({waypoint.north:.1f}, {waypoint.east:.1f}, {waypoint.down:.1f}), '
            f'offboard={self.is_offboard()}, armed={self.is_armed()}'
        )

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

    def nav_state_text(self) -> str:
        if self.status is None:
            return 'none'
        return str(self.status.nav_state)

    def arming_state_text(self) -> str:
        if self.status is None:
            return 'none'
        return str(self.status.arming_state)

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

    def publish_mapping_active(self, active: bool) -> None:
        msg = Bool()
        msg.data = bool(active)
        self.mapping_active_pub.publish(msg)

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

    def set_auto_land_mode(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=4.0,
            param3=6.0,
        )
        self.get_logger().info('Requested PX4 auto land mode.')

    def land(self) -> None:
        self.set_auto_land_mode()
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
