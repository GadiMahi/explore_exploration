from typing import List, Tuple
import math
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)

def grid_to_world(map_info: OccupancyGrid, col: int, row: int) -> Tuple[float, float]:
    res = map_info.info.resolution
    origin= map_info.info.origin 
    ox, oy = origin.position.x, origin.position.y
    q = origin.orientation 
    yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)


    wx = ox+ (col + 0.5) * res
    wy = oy+ (row) * res

    return wx, wy

def world_to_grid(map_info: OccupancyGrid, x:float, y:float):

    res = map_info.info.resolution
    origin= map_info.info.origin
    ox, oy = origin.position.x, origin.position.y
    q = origin.orientation
    yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

    dx, dy = x-ox, y-oy

    cos_y, sin_y = math.cos(yaw), math.sin(yaw)  
    lx = dx * cos_y + dy * sin_y
    ly = -dx * sin_y + dy * cos_y

    col = int(lx // res)  
    row = int(ly // res)
    return col, row

def _idx(width: int, col: int, row: int) -> int:
    return row * width + col    

def _cell_value(data: List[int], width: int, height: int, col: int, row: int) -> int | None:
    """Return occupancy value or None if out of bounds."""
    if col < 0 or row < 0 or col >= width or row >= height:
        return None
    return data[_idx(width, col, row)]

def detect_frontier_cells(og: OccupancyGrid,
                          free_value: int = 0,
                          unknown_value: int = -1) -> List[Tuple[int, int]]:
    """
    Return a list of frontier CELL coordinates as (col, row).
    Frontier definition (4-connected):
      - Cell is FREE
      - At least one 4-neighbor is UNKNOWN
    """
    width = og.info.width
    height = og.info.height
    data = og.data  # flat list (row-major)

    frontiers: List[Tuple[int, int]] = []

    for row in range(height):
        for col in range(width):
            v = data[_idx(width, col, row)]
            if v != free_value:
                continue

            # 4-connected neighbors
            neighbors = [
                (col + 1, row),
                (col - 1, row),
                (col, row + 1),
                (col, row - 1),
            ]

            is_frontier = False
            for nc, nr in neighbors:
                nv = _cell_value(data, width, height, nc, nr)
                if nv is None:
                    continue
                if nv is not None and nv<0:
                    is_frontier = True
                    break

            if is_frontier:
                frontiers.append((col, row))

    return frontiers
