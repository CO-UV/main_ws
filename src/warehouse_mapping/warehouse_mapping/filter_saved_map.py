import argparse
import math
import os
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple


@dataclass(frozen=True)
class MapMeta:
    image_path: str
    resolution: float
    origin: str
    negate: int
    occupied_thresh: float
    free_thresh: float


def parse_simple_yaml(path: str) -> MapMeta:
    path = os.path.expanduser(path)
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
    return MapMeta(
        image_path=image_path,
        resolution=float(values['resolution']),
        origin=values.get('origin', '[0, 0, 0]'),
        negate=int(values.get('negate', '0')),
        occupied_thresh=float(values.get('occupied_thresh', '0.65')),
        free_thresh=float(values.get('free_thresh', '0.25')),
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
            raise ValueError(f'Only max_value=255 is supported, got {max_value}')

        data = stream.read(width * height)
        if len(data) != width * height:
            raise ValueError(f'PGM data size mismatch: expected {width * height}, got {len(data)}')

    return width, height, list(data)


def write_pgm(path: str, width: int, height: int, data: List[int]) -> None:
    with open(path, 'wb') as stream:
        header = f'P5\n# CREATOR: warehouse_mapping filter_saved_map\n{width} {height}\n255\n'
        stream.write(header.encode('ascii'))
        stream.write(bytes(data))


def write_yaml(path: str, image_name: str, meta: MapMeta) -> None:
    content = (
        f'image: {image_name}\n'
        f'mode: trinary\n'
        f'resolution: {meta.resolution:.8f}\n'
        f'origin: {meta.origin}\n'
        f'negate: {meta.negate}\n'
        f'occupied_thresh: {meta.occupied_thresh:.6f}\n'
        f'free_thresh: {meta.free_thresh:.6f}\n'
    )
    with open(path, 'w', encoding='utf-8') as stream:
        stream.write(content)


def classify_pixels(
    pixels: List[int],
    occupied_pixel: int,
    free_pixel: int,
    unknown_mode: str,
) -> List[int]:
    grid = []
    for pixel in pixels:
        if pixel <= occupied_pixel:
            grid.append(100)
        elif pixel >= free_pixel:
            grid.append(0)
        elif unknown_mode == 'occupied':
            grid.append(100)
        elif unknown_mode == 'free':
            grid.append(0)
        else:
            grid.append(-1)
    return grid


def radius_to_cells(radius_m: float, resolution: float) -> int:
    return max(0, int(round(radius_m / resolution)))


def dilate_occupied(grid: List[int], width: int, height: int, radius: int) -> List[bool]:
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


def erode_occupied(occupied: List[bool], width: int, height: int, radius: int) -> List[bool]:
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


def close_occupied(grid: List[int], width: int, height: int, radius: int) -> List[int]:
    if radius <= 0:
        return grid
    closed = erode_occupied(dilate_occupied(grid, width, height, radius), width, height, radius)
    return [100 if closed[index] else value for index, value in enumerate(grid)]


def dilate_occupied_mask(occupied: List[bool], width: int, height: int, radius: int) -> List[bool]:
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


def open_occupied(grid: List[int], width: int, height: int, radius: int) -> List[int]:
    if radius <= 0:
        return grid
    occupied = [value == 100 for value in grid]
    eroded = erode_occupied(occupied, width, height, radius)
    opened = dilate_occupied_mask(eroded, width, height, radius)
    return [100 if opened[index] else 0 if value == 100 else value for index, value in enumerate(grid)]


def remove_small_components(grid: List[int], width: int, height: int, min_cells: int) -> List[int]:
    if min_cells <= 1:
        return grid

    result = list(grid)
    visited = [False] * len(grid)
    offsets = (
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
            for dx, dy in offsets:
                x = cx + dx
                y = cy + dy
                if not (0 <= x < width and 0 <= y < height):
                    continue
                neighbor = y * width + x
                if visited[neighbor] or grid[neighbor] != 100:
                    continue
                visited[neighbor] = True
                queue.append(neighbor)

        if len(component) < min_cells:
            for index in component:
                result[index] = 0

    return result


def grid_to_pgm(grid: List[int]) -> List[int]:
    output = []
    for value in grid:
        if value == 100:
            output.append(0)
        elif value < 0:
            output.append(205)
        else:
            output.append(254)
    return output


def default_output_prefix(input_yaml: str) -> str:
    path = os.path.expanduser(input_yaml)
    stem, _ = os.path.splitext(path)
    return f'{stem}_filtered'


def main() -> None:
    parser = argparse.ArgumentParser(description='Filter a saved nav2/RTAB occupancy PGM/YAML map.')
    parser.add_argument('--input-yaml', default='~/main_ws/maps/warehouse_map.yaml')
    parser.add_argument('--output-prefix', default='')
    parser.add_argument('--occupied-pixel', type=int, default=60)
    parser.add_argument('--free-pixel', type=int, default=205)
    parser.add_argument('--unknown-mode', choices=['keep', 'free', 'occupied'], default='keep')
    parser.add_argument('--occupied-open-radius', type=float, default=0.10)
    parser.add_argument('--occupied-close-radius', type=float, default=0.0)
    parser.add_argument('--min-occupied-component-area', type=float, default=0.25)
    args = parser.parse_args()

    input_yaml = os.path.expanduser(args.input_yaml)
    output_prefix = os.path.expanduser(args.output_prefix) if args.output_prefix else default_output_prefix(input_yaml)
    meta = parse_simple_yaml(input_yaml)
    width, height, pixels = read_pgm(meta.image_path)

    grid = classify_pixels(pixels, args.occupied_pixel, args.free_pixel, args.unknown_mode)
    open_cells = radius_to_cells(args.occupied_open_radius, meta.resolution)
    grid = open_occupied(grid, width, height, open_cells)

    close_cells = radius_to_cells(args.occupied_close_radius, meta.resolution)
    grid = close_occupied(grid, width, height, close_cells)

    min_cells = max(
        0,
        int(math.ceil(args.min_occupied_component_area / (meta.resolution * meta.resolution))),
    )
    grid = remove_small_components(grid, width, height, min_cells)

    directory = os.path.dirname(output_prefix)
    if directory:
        os.makedirs(directory, exist_ok=True)

    pgm_path = f'{output_prefix}.pgm'
    yaml_path = f'{output_prefix}.yaml'
    write_pgm(pgm_path, width, height, grid_to_pgm(grid))
    write_yaml(yaml_path, os.path.basename(pgm_path), meta)
    print(f'Saved filtered map: {yaml_path} / {pgm_path}')


if __name__ == '__main__':
    main()
