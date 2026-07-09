import os
from collections import deque
from typing import List
from typing import Sequence

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class SaveOccupancyMap(Node):
    def __init__(self) -> None:
        super().__init__('save_occupancy_map')

        self.map_topic = str(self.declare_parameter('map_topic', '/rtabmap/grid_prob_map').value)
        self.extra_map_topics = [
            str(topic)
            for topic in self.declare_parameter(
                'extra_map_topics',
                [],
            ).value
        ]
        self.output_prefix = os.path.expanduser(
            str(self.declare_parameter('output_prefix', '~/main_ws/maps/warehouse_map').value)
        )
        self.free_thresh = float(self.declare_parameter('free_thresh', 0.25).value)
        self.occupied_thresh = float(self.declare_parameter('occupied_thresh', 0.55).value)
        self.trinary = parse_bool(self.declare_parameter('trinary', False).value)
        self.occupied_close_radius = float(
            self.declare_parameter('occupied_close_radius', 0.0).value
        )
        self.occupied_open_radius = float(
            self.declare_parameter('occupied_open_radius', 0.10).value
        )
        self.min_occupied_component_area = float(
            self.declare_parameter('min_occupied_component_area', 0.25).value
        )
        self.negate = int(self.declare_parameter('negate', 0).value)
        self.saved = False

        qos_profiles = [
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        ]
        self.map_topics = self.unique_topics([self.map_topic] + self.extra_map_topics)
        self.map_subscriptions = []
        for topic in self.map_topics:
            for qos in qos_profiles:
                self.map_subscriptions.append(
                    self.create_subscription(
                        OccupancyGrid,
                        topic,
                        lambda msg, topic=topic: self.map_cb(msg, topic),
                        qos,
                    )
                )
        self.diagnostic_timer = self.create_timer(3.0, self.log_diagnostics)

        self.get_logger().info(
            f'Waiting for OccupancyGrid on {self.map_topics}; output={self.output_prefix}.yaml'
        )

    def unique_topics(self, topics: Sequence[str]) -> List[str]:
        result = []
        for topic in topics:
            topic = topic.strip()
            if not topic:
                continue
            if not topic.startswith('/'):
                topic = f'/{topic}'
            if topic not in result:
                result.append(topic)
        return result

    def log_diagnostics(self) -> None:
        if self.saved:
            return
        topic_types = dict(self.get_topic_names_and_types())
        status = []
        for topic in self.map_topics:
            types = ','.join(topic_types.get(topic, [])) or 'no_type'
            publishers = self.count_publishers(topic)
            status.append(f'{topic}: pubs={publishers}, type={types}')
        self.get_logger().warn(
            'Still waiting for a non-empty OccupancyGrid. ' + '; '.join(status)
        )

    def map_cb(self, msg: OccupancyGrid, topic: str) -> None:
        if self.saved:
            return
        if msg.info.width == 0 or msg.info.height == 0:
            self.get_logger().warn(f'Received empty map from {topic}, waiting for a non-empty map.')
            return

        self.saved = True
        self.save_map(msg)
        self.get_logger().info(
            f'Saved map from {topic}: {self.output_prefix}.yaml / {self.output_prefix}.pgm'
        )
        rclpy.shutdown()

    def save_map(self, msg: OccupancyGrid) -> None:
        directory = os.path.dirname(self.output_prefix)
        if directory:
            os.makedirs(directory, exist_ok=True)

        pgm_path = f'{self.output_prefix}.pgm'
        yaml_path = f'{self.output_prefix}.yaml'
        pgm_data = self.make_pgm_data(msg)

        with open(pgm_path, 'wb') as stream:
            header = f'P5\n# CREATOR: warehouse_mapping save_occupancy_map\n{msg.info.width} {msg.info.height}\n255\n'
            stream.write(header.encode('ascii'))
            stream.write(bytes(pgm_data))

        image_name = os.path.basename(pgm_path)
        origin = msg.info.origin
        mode = 'trinary' if self.trinary else 'scale'
        yaml = (
            f'image: {image_name}\n'
            f'mode: {mode}\n'
            f'resolution: {msg.info.resolution:.8f}\n'
            f'origin: [{origin.position.x:.8f}, {origin.position.y:.8f}, 0.00000000]\n'
            f'negate: {self.negate}\n'
            f'occupied_thresh: {self.occupied_thresh:.6f}\n'
            f'free_thresh: {self.free_thresh:.6f}\n'
        )
        with open(yaml_path, 'w', encoding='utf-8') as stream:
            stream.write(yaml)

    def make_pgm_data(self, msg: OccupancyGrid) -> List[int]:
        width = int(msg.info.width)
        height = int(msg.info.height)
        occupied_threshold = int(self.occupied_thresh * 100.0)
        free_threshold = int(self.free_thresh * 100.0)
        close_cells = max(0, int(round(self.occupied_close_radius / msg.info.resolution)))
        open_cells = max(0, int(round(self.occupied_open_radius / msg.info.resolution)))

        grid: List[int] = []
        for value in msg.data:
            value = int(value)
            if value < 0:
                grid.append(-1)
            elif value >= occupied_threshold:
                grid.append(100)
            elif value <= free_threshold:
                grid.append(0)
            elif self.trinary:
                grid.append(100)
            else:
                grid.append(-1)

        if open_cells > 0:
            grid = self.open_occupied(grid, width, height, open_cells)

        if close_cells > 0:
            closed = self.erode_occupied(
                self.dilate_occupied(grid, width, height, close_cells),
                width,
                height,
                close_cells,
            )
            grid = [
                100 if closed[index] else value
                for index, value in enumerate(grid)
            ]

        min_component_cells = max(
            0,
            int(round(self.min_occupied_component_area / (msg.info.resolution * msg.info.resolution))),
        )
        if min_component_cells > 1:
            grid = self.remove_small_occupied_components(grid, width, height, min_component_cells)

        output: List[int] = []
        for image_y in range(height - 1, -1, -1):
            row_offset = image_y * width
            for x in range(width):
                value = grid[row_offset + x]
                if value < 0:
                    output.append(205)
                elif value >= 100:
                    output.append(0)
                else:
                    output.append(254)
        return output

    def open_occupied(self, grid: List[int], width: int, height: int, radius: int) -> List[int]:
        occupied = [value == 100 for value in grid]
        eroded = self.erode_occupied(occupied, width, height, radius)
        opened = self.dilate_occupied_mask(eroded, width, height, radius)
        return [
            100 if opened[index] else 0 if value == 100 else value
            for index, value in enumerate(grid)
        ]

    def dilate_occupied(self, grid: List[int], width: int, height: int, radius: int) -> List[bool]:
        result = [False] * len(grid)
        for index, value in enumerate(grid):
            if value != 100:
                continue
            cx = index % width
            cy = index // width
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if dx * dx + dy * dy > radius * radius:
                        continue
                    x = cx + dx
                    y = cy + dy
                    if 0 <= x < width and 0 <= y < height:
                        result[y * width + x] = True
        return result

    def dilate_occupied_mask(self, occupied: List[bool], width: int, height: int, radius: int) -> List[bool]:
        result = [False] * len(occupied)
        for index, value in enumerate(occupied):
            if not value:
                continue
            cx = index % width
            cy = index // width
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if dx * dx + dy * dy > radius * radius:
                        continue
                    x = cx + dx
                    y = cy + dy
                    if 0 <= x < width and 0 <= y < height:
                        result[y * width + x] = True
        return result

    def erode_occupied(self, occupied: List[bool], width: int, height: int, radius: int) -> List[bool]:
        result = [False] * len(occupied)
        for index, value in enumerate(occupied):
            if not value:
                continue
            cx = index % width
            cy = index // width
            keep = True
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if dx * dx + dy * dy > radius * radius:
                        continue
                    x = cx + dx
                    y = cy + dy
                    if not (0 <= x < width and 0 <= y < height) or not occupied[y * width + x]:
                        keep = False
                        break
                if not keep:
                    break
            result[index] = keep
        return result

    def remove_small_occupied_components(
        self,
        grid: List[int],
        width: int,
        height: int,
        min_cells: int,
    ) -> List[int]:
        result = list(grid)
        visited = [False] * len(grid)
        neighbor_offsets = (
            (-1, -1), (0, -1), (1, -1),
            (-1, 0), (1, 0),
            (-1, 1), (0, 1), (1, 1),
        )

        for start_index, value in enumerate(grid):
            if value != 100 or visited[start_index]:
                continue

            component = []
            queue = deque([start_index])
            visited[start_index] = True

            while queue:
                index = queue.popleft()
                component.append(index)
                cx = index % width
                cy = index // width

                for dx, dy in neighbor_offsets:
                    x = cx + dx
                    y = cy + dy
                    if not (0 <= x < width and 0 <= y < height):
                        continue
                    neighbor_index = y * width + x
                    if visited[neighbor_index] or grid[neighbor_index] != 100:
                        continue
                    visited[neighbor_index] = True
                    queue.append(neighbor_index)

            if len(component) < min_cells:
                for index in component:
                    result[index] = 0

        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SaveOccupancyMap()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
