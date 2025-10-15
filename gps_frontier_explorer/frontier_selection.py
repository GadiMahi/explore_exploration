import math
from typing import Iterable, Optional, Sequence, Tuple

Point = Tuple[float, float]


def _too_close(p: Point, q: Point, radius: float) -> bool:
    """Return True if points p and q are within radius distance."""
    return (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 <= radius ** 2


def select_best_centroid(
    centroids: Sequence[Point],
    goal_x: float,
    goal_y: float,
    *,
    last_goal: Optional[Point] = None,
    visited: Optional[Iterable[Point]] = None,
    skip_radius: float = 0.30,
    tie_threshold: float = 0.10,
    robot_xy: Optional[Point] = None,
    max_goal_dist: Optional[float] = None,
) -> Optional[Tuple[float, float, float, Optional[float]]]:
    """
    Choose the best centroid toward the goal while skipping previously visited or duplicate ones.
    Returns (cx, cy, d_goal, d_robot) or None if no valid candidate.
    """

    if not centroids:
        return None

    candidates: list[Tuple[float, float, float, Optional[float]]] = []

    for (cx, cy) in centroids:
        # Skip if too close to last goal
        if last_goal is not None and _too_close((cx, cy), last_goal, skip_radius):
            continue

        # Skip if too close to any visited frontier
        if visited:
            skip = any(_too_close((cx, cy), v, skip_radius) for v in visited)
            if skip:
                continue

        d_goal = math.hypot(cx - goal_x, cy - goal_y)
        if max_goal_dist is not None and d_goal > max_goal_dist:
            continue

        d_robot = math.hypot(cx - robot_xy[0], cy - robot_xy[1]) if robot_xy else None
        candidates.append((cx, cy, d_goal, d_robot))

    if not candidates:
        return None

    best = None
    best_d_goal = float("inf")
    best_d_robot = float("inf")

    for c in candidates:
        cx, cy, d_goal, d_robot = c
        if d_goal < best_d_goal - tie_threshold:
            best, best_d_goal, best_d_robot = c, d_goal, d_robot if d_robot else float("inf")
        elif abs(d_goal - best_d_goal) <= tie_threshold:
            if d_robot is not None and d_robot < best_d_robot - 1e-9:
                best, best_d_goal, best_d_robot = c, d_goal, d_robot

    return best