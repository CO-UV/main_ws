"""Pure-pursuit path follower for the warehouse UGV.

Subscribes to a nav_msgs/Path (typically /planned_path from ugv_path_planner)
and the UGV odometry (/ugv/odom, published by the gz OdometryPublisher
plugin), and drives the vehicle to the goal by publishing geometry_msgs/Twist
on /ugv/cmd_vel.

Frame handling: unlike the DiffDrive plugin's wheel odometry (which starts at
(0, 0, yaw=0) relative to the spawn pose), the gz OdometryPublisher plugin
used here for ground-truth odometry reports the model's pose directly in the
world frame -- confirmed by spawning at a non-zero pose and observing
/ugv/odom start at that same absolute position. The path is also expressed in
the world/map frame, so /ugv/odom can be compared to it directly with no
spawn offset.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PurePursuit(Node):
    def __init__(self) -> None:
        super().__init__('ugv_pure_pursuit')

        self.path_topic = str(self.declare_parameter('path_topic', '/planned_path').value)
        self.odom_topic = str(self.declare_parameter('odom_topic', '/ugv/odom').value)
        self.cmd_topic = str(self.declare_parameter('cmd_topic', '/ugv/cmd_vel').value)
        # Keep the corner-cutting radius within the A* planner's own safety
        # margin (robot_radius + obstacle_padding, 0.55 m by default -- see
        # astar_planner.launch.py). A larger lookahead cuts corners wider than
        # the path's inflation buffer and can clip obstacles the path was
        # routed around (observed: 1.2 m drove the UGV straight into a shelf
        # the A* path detoured around).
        self.lookahead = float(self.declare_parameter('lookahead', 0.5).value)
        self.max_linear = float(self.declare_parameter('max_linear', 1.0).value)
        self.max_angular = float(self.declare_parameter('max_angular', 2.0).value)
        self.goal_tolerance = float(self.declare_parameter('goal_tolerance', 0.4).value)
        self.k_angular = float(self.declare_parameter('k_angular', 2.5).value)
        # Above this heading error (rad) the robot turns in place; below it the
        # forward speed is scaled down but never below min_speed_ratio.
        self.turn_in_place_angle = float(self.declare_parameter('turn_in_place_angle', 1.4).value)
        self.min_speed_ratio = float(self.declare_parameter('min_speed_ratio', 0.3).value)

        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)

        self.path_xy = []
        self.nearest_index = 0
        self.finished = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(Path, self.path_topic, self.path_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)

        self.get_logger().info(
            f'Pure pursuit ready: path={self.path_topic}, odom={self.odom_topic}, '
            f'cmd={self.cmd_topic}, lookahead={self.lookahead} m, '
            f'max_linear={self.max_linear} m/s'
        )

    def path_cb(self, msg: Path) -> None:
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not pts:
            return
        # Only (re)load a new path; keep tracking progress on repeated messages.
        if pts != self.path_xy:
            self.path_xy = pts
            self.nearest_index = 0
            self.finished = False
            self.get_logger().info(f'Loaded path with {len(pts)} points.')

    def odom_cb(self, msg: Odometry) -> None:
        wx = msg.pose.pose.position.x
        wy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        # Broadcast map -> base_link directly (map == world here) so RViz can
        # show the UGV pose against the planned path and occupancy map even
        # before a path has been loaded.
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = wx
        tf.transform.translation.y = wy
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = q
        self.tf_broadcaster.sendTransform(tf)

        if not self.path_xy:
            return

        goal = self.path_xy[-1]
        dist_to_goal = math.hypot(goal[0] - wx, goal[1] - wy)
        if dist_to_goal <= self.goal_tolerance:
            if not self.finished:
                self.finished = True
                self.get_logger().info(
                    f'Goal reached (dist={dist_to_goal:.2f} m). Stopping UGV.'
                )
            self.cmd_pub.publish(Twist())
            return

        # Advance the nearest-point pointer (monotonic, never goes backwards).
        best_i = self.nearest_index
        best_d = float('inf')
        for i in range(self.nearest_index, len(self.path_xy)):
            d = math.hypot(self.path_xy[i][0] - wx, self.path_xy[i][1] - wy)
            if d < best_d:
                best_d = d
                best_i = i
            # stop scanning once we clearly move away from the robot
            if d > best_d + 2.0 * self.lookahead:
                break
        self.nearest_index = best_i

        target = self.find_lookahead_target(wx, wy)

        heading_to_target = math.atan2(target[1] - wy, target[0] - wx)
        heading_error = normalize_angle(heading_to_target - yaw)

        cmd = Twist()
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, self.k_angular * heading_error))
        # Keep making forward progress while steering: only stop to turn in place
        # when the target is well off-heading; otherwise scale speed down to a
        # floor so the robot never crawls to a standstill on a wiggly A* path.
        abs_err = abs(heading_error)
        if abs_err >= self.turn_in_place_angle:
            turn_factor = 0.0
        else:
            turn_factor = max(self.min_speed_ratio, 1.0 - abs_err / self.turn_in_place_angle)
        cmd.linear.x = self.max_linear * turn_factor
        self.cmd_pub.publish(cmd)

    def find_lookahead_target(self, wx: float, wy: float):
        """Geometric circle/segment intersection for the lookahead target.

        Picking the first WAYPOINT whose distance to the robot is >=
        lookahead (the previous approach) flickers between two waypoints
        whenever one sits almost exactly at the lookahead distance -- which
        happens routinely here because the A* path collapses long straight
        runs into a single big segment (e.g. two waypoints 4 m apart). A tiny
        position change then toggles the target between that near waypoint
        (which can already be slightly behind the robot) and the next one far
        ahead, commanding a ~180 degree heading flip every callback.
        Interpolating the exact point where the path crosses the lookahead
        circle instead varies continuously with robot motion, eliminating the
        flip-flop.

        Fallback: if the robot is currently farther from every remaining
        segment than the lookahead radius (e.g. it cut a corner and briefly
        ended up more than `lookahead` off the path), NO circle intersection
        exists. Falling back to the final goal here (as a naive implementation
        does) makes the robot abandon the whole planned route and drive
        straight at the goal in a straight line -- observed in practice
        driving the UGV through a shelf the path had detoured around, and the
        resulting divergence from the path then keeps the nearest-index search
        from ever recovering. Falling back to the nearest path point instead
        steers the robot back onto the route.
        """
        n = len(self.path_xy)
        for i in range(self.nearest_index, n - 1):
            ax, ay = self.path_xy[i]
            bx, by = self.path_xy[i + 1]
            dx, dy = bx - ax, by - ay
            fx, fy = ax - wx, ay - wy
            a = dx * dx + dy * dy
            if a < 1e-9:
                continue
            b = 2.0 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - self.lookahead * self.lookahead
            disc = b * b - 4.0 * a * c
            if disc < 0.0:
                continue
            disc_sqrt = math.sqrt(disc)
            t_exit = (-b + disc_sqrt) / (2.0 * a)
            t_enter = (-b - disc_sqrt) / (2.0 * a)
            if 0.0 <= t_exit <= 1.0:
                return (ax + t_exit * dx, ay + t_exit * dy)
            if 0.0 <= t_enter <= 1.0:
                return (ax + t_enter * dx, ay + t_enter * dy)
        return self.path_xy[self.nearest_index]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
