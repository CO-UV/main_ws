import heapq
import math
import os
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


GridCell = Tuple[int, int]


@dataclass(frozen=True)
class MapMeta:
    image_path: str
    resolution: float
    origin_x: float
    origin_y: float
    occupied_thresh: float
    free_thresh: float
    negate: int


def parse_simple_yaml(path: str) -> MapMeta:
    values = {}
    with open(path, 'r', encoding='utf-8') as stream:
        for raw_line in stream:
            line = raw_line.strip()
            if not line or line.startswith('#') or ':' not in line:
                continue
            key, value = line.split(':', 1)
            values[key.strip()] = value.strip()

    base_dir = os.path.dirname(os.path.abspath(path))
    image_value = values['image']
    image_path = image_value if os.path.isabs(image_value) else os.path.join(base_dir, image_value)
    origin = values.get('origin', '[0, 0, 0]').strip().strip('[]')
    origin_values = [float(item.strip()) for item in origin.split(',')]

    return MapMeta(
        image_path=image_path,
        resolution=float(values['resolution']),
        origin_x=float(origin_values[0]),
        origin_y=float(origin_values[1]),
        occupied_thresh=float(values.get('occupied_thresh', '0.65')),
        free_thresh=float(values.get('free_thresh', '0.25')),
        negate=int(values.get('negate', '0')),
    )


def read_pgm(path: str) -> Tuple[int, int, List[int]]:
    with open(path, 'rb') as stream:
        magic = stream.readline().strip()
        if magic != b'P5':
            raise ValueError(f'Only binary PGM P5 is supported, got {magic!r}')

        tokens: List[bytes] = []
        while len(tokens) < 3:
            line = stream.readline()
            if not line:
                raise ValueError('Unexpected end of PGM header.')
            line = line.split(b'#', 1)[0]
            tokens.extend(line.split())

        width = int(tokens[0])
        height = int(tokens[1])
        max_value = int(tokens[2])
        if max_value != 255:
            raise ValueError(f'Only max_value=255 PGM is supported, got {max_value}')

        data = stream.read(width * height)
        if len(data) != width * height:
            raise ValueError(f'PGM data size mismatch: expected {width * height}, got {len(data)}')

    return width, height, list(data)


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class AStarPlanner(Node):
    def __init__(self) -> None:
        super().__init__('astar_planner')

        default_map_yaml = os.path.expanduser('~/maps/warehouse_occupancy.yaml')
        self.map_yaml = os.path.expanduser(
            str(
                self.declare_parameter(
                    'map_yaml',
                    default_map_yaml,
                ).value
            )
        )
        self.start_x = float(self.declare_parameter('start_x', 0.0).value)
        self.start_y = float(self.declare_parameter('start_y', -10.0).value)
        self.goal_x = float(self.declare_parameter('goal_x', 9.0).value)
        self.goal_y = float(self.declare_parameter('goal_y', 9.0).value)
        self.robot_radius = float(self.declare_parameter('robot_radius', 0.35).value)
        self.obstacle_padding = float(self.declare_parameter('obstacle_padding', 0.20).value)
        self.clearance_radius = float(self.declare_parameter('clearance_radius', 1.00).value)
        self.clearance_weight = float(self.declare_parameter('clearance_weight', 3.00).value)
        self.unknown_is_occupied = parse_bool(
            self.declare_parameter('unknown_is_occupied', True).value
        )
        self.frame_id = str(self.declare_parameter('frame_id', 'map').value)
        self.publish_period = float(self.declare_parameter('publish_period', 1.0).value)

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/planner/inflated_map', 1)
        self.marker_pub = self.create_publisher(Marker, '/planner/start_goal_markers', 10)

        self.meta = parse_simple_yaml(self.map_yaml)
        self.width, self.height, self.pixels = read_pgm(self.meta.image_path)
        self.occupancy = self.make_occupancy_grid()
        self.inflated = self.inflate_obstacles(self.occupancy)
        self.clearance_distances = self.compute_clearance_distances(self.inflated)
        self.path = self.plan_path()

        self.timer = self.create_timer(self.publish_period, self.publish_outputs)
        self.publish_outputs()

    def make_occupancy_grid(self) -> List[int]:
        grid: List[int] = []
        for grid_y in range(self.height):
            image_y = self.height - 1 - grid_y
            row_offset = image_y * self.width
            for x in range(self.width):
                value = self.pixels[row_offset + x]
                if self.meta.negate:
                    occ_probability = value / 255.0
                else:
                    occ_probability = (255 - value) / 255.0

                if occ_probability > self.meta.occupied_thresh:
                    grid.append(100)
                elif occ_probability < self.meta.free_thresh:
                    grid.append(0)
                else:
                    grid.append(-1)
        return grid

    def inflate_obstacles(self, occupancy: List[int]) -> List[int]:
        inflated = list(occupancy)
        inflation_radius = self.robot_radius + self.obstacle_padding
        radius_cells = int(math.ceil(inflation_radius / self.meta.resolution))
        self.get_logger().info(
            f'Inflating obstacles by {inflation_radius:.2f} m '
            f'(robot_radius={self.robot_radius:.2f}, padding={self.obstacle_padding:.2f})'
        )
        obstacle_cells = [
            (index % self.width, index // self.width)
            for index, value in enumerate(occupancy)
            if value == 100 or (self.unknown_is_occupied and value == -1)
        ]

        for ox, oy in obstacle_cells:
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy > radius_cells * radius_cells:
                        continue
                    x = ox + dx
                    y = oy + dy
                    if self.in_bounds((x, y)):
                        inflated[self.to_index((x, y))] = 100
        return inflated

    def compute_clearance_distances(self, occupancy: List[int]) -> List[float]:
        max_distance_cells = max(1, int(math.ceil(self.clearance_radius / self.meta.resolution)))
        max_distance = max_distance_cells * self.meta.resolution
        distances = [math.inf] * len(occupancy)
        heap: List[Tuple[float, GridCell]] = []

        for index, value in enumerate(occupancy):
            if value == 100:
                cell = (index % self.width, index // self.width)
                distances[index] = 0.0
                heapq.heappush(heap, (0.0, cell))

        while heap:
            distance, cell = heapq.heappop(heap)
            if distance > max_distance:
                continue
            if distance > distances[self.to_index(cell)]:
                continue

            for neighbor, step_cost in self.clearance_neighbors(cell):
                new_distance = distance + step_cost * self.meta.resolution
                index = self.to_index(neighbor)
                if new_distance < distances[index] and new_distance <= max_distance:
                    distances[index] = new_distance
                    heapq.heappush(heap, (new_distance, neighbor))

        self.get_logger().info(
            f'Clearance cost enabled: radius={self.clearance_radius:.2f} m, '
            f'weight={self.clearance_weight:.2f}'
        )
        return distances

    def plan_path(self) -> List[GridCell]:
        start = self.world_to_grid(self.start_x, self.start_y)
        goal = self.world_to_grid(self.goal_x, self.goal_y)

        if not self.is_free(start):
            raise RuntimeError(f'Start is not in free space: world=({self.start_x}, {self.start_y}), grid={start}')
        if not self.is_free(goal):
            raise RuntimeError(f'Goal is not in free space: world=({self.goal_x}, {self.goal_y}), grid={goal}')

        open_heap: List[Tuple[float, float, GridCell]] = []
        heapq.heappush(open_heap, (self.heuristic(start, goal), 0.0, start))
        came_from: Dict[GridCell, GridCell] = {}
        cost_so_far: Dict[GridCell, float] = {start: 0.0}

        while open_heap:
            _, current_cost, current = heapq.heappop(open_heap)
            if current == goal:
                path = self.reconstruct_path(came_from, current)
                self.get_logger().info(
                    f'A* path found: {len(path)} cells, cost={current_cost:.2f}, '
                    f'start={start}, goal={goal}'
                )
                return path

            for neighbor, move_cost in self.neighbors(current):
                new_cost = cost_so_far[current] + move_cost + self.clearance_penalty(neighbor)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (priority, new_cost, neighbor))
                    came_from[neighbor] = current

        raise RuntimeError(f'No path found from {start} to {goal}')

    def reconstruct_path(self, came_from: Dict[GridCell, GridCell], current: GridCell) -> List[GridCell]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return self.simplify_path(path)

    def simplify_path(self, path: List[GridCell]) -> List[GridCell]:
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        previous_direction: Optional[GridCell] = None
        for a, b in zip(path, path[1:]):
            direction = (b[0] - a[0], b[1] - a[1])
            if previous_direction is not None and direction != previous_direction:
                simplified.append(a)
            previous_direction = direction
        simplified.append(path[-1])
        return simplified

    def neighbors(self, cell: GridCell) -> Iterable[Tuple[GridCell, float]]:
        x, y = cell
        for dx, dy, cost in (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ):
            neighbor = (x + dx, y + dy)
            if self.is_free(neighbor):
                yield neighbor, cost

    def clearance_neighbors(self, cell: GridCell) -> Iterable[Tuple[GridCell, float]]:
        x, y = cell
        for dx, dy, cost in (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ):
            neighbor = (x + dx, y + dy)
            if self.in_bounds(neighbor):
                yield neighbor, cost

    def clearance_penalty(self, cell: GridCell) -> float:
        if self.clearance_weight <= 0.0 or self.clearance_radius <= 0.0:
            return 0.0

        distance = self.clearance_distances[self.to_index(cell)]
        if math.isinf(distance) or distance >= self.clearance_radius:
            return 0.0

        normalized = (self.clearance_radius - distance) / self.clearance_radius
        return self.clearance_weight * normalized

    def heuristic(self, a: GridCell, b: GridCell) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def is_free(self, cell: GridCell) -> bool:
        if not self.in_bounds(cell):
            return False
        return self.inflated[self.to_index(cell)] == 0

    def in_bounds(self, cell: GridCell) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def to_index(self, cell: GridCell) -> int:
        return cell[1] * self.width + cell[0]

    def world_to_grid(self, x: float, y: float) -> GridCell:
        return (
            int((x - self.meta.origin_x) / self.meta.resolution),
            int((y - self.meta.origin_y) / self.meta.resolution),
        )

    def grid_to_world(self, cell: GridCell) -> Tuple[float, float]:
        return (
            self.meta.origin_x + (cell[0] + 0.5) * self.meta.resolution,
            self.meta.origin_y + (cell[1] + 0.5) * self.meta.resolution,
        )

    def publish_outputs(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.make_path_msg(stamp))
        self.map_pub.publish(self.make_inflated_map_msg(stamp))
        self.marker_pub.publish(self.make_marker(stamp, self.start_x, self.start_y, 0, 0.1, 0.8, 0.1))
        self.marker_pub.publish(self.make_marker(stamp, self.goal_x, self.goal_y, 1, 0.9, 0.1, 0.1))

    def make_path_msg(self, stamp) -> Path:
        msg = Path()
        msg.header = Header(stamp=stamp, frame_id=self.frame_id)
        for cell in self.path:
            x, y = self.grid_to_world(cell)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.05
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def make_inflated_map_msg(self, stamp) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header = Header(stamp=stamp, frame_id=self.frame_id)
        msg.info.resolution = self.meta.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.meta.origin_x
        msg.info.origin.position.y = self.meta.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.inflated
        return msg

    def make_marker(self, stamp, x: float, y: float, marker_id: int, r: float, g: float, b: float) -> Marker:
        marker = Marker()
        marker.header = Header(stamp=stamp, frame_id=self.frame_id)
        marker.ns = 'astar_start_goal'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.45
        marker.scale.y = 0.45
        marker.scale.z = 0.45
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        return marker


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AStarPlanner()
        rclpy.spin(node)
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f'astar_planner failed: {exc}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
