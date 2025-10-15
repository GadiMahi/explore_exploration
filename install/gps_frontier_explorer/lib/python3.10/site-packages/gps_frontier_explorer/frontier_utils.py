import math
import numpy as np
from sklearn.cluster import DBSCAN
from gps_frontier_explorer.frontier_detection import grid_to_world
from nav_msgs.msg import OccupancyGrid

def cluster_frontiers(frontier_cells, og, eps: float = None, min_samples: int = 3,
                      min_cluster_size: int = 3, max_cluster_points: int = 80):
    """
    Cluster frontier cells into spatial groups using adaptive DBSCAN,
    automatically handling different map scales and splitting oversized clusters.

    Args:
        frontier_cells: list of (col, row) grid coordinates.
        og: OccupancyGrid message.
        eps: optional fixed neighborhood distance (auto if None).
        min_samples: DBSCAN neighborhood density threshold.
        min_cluster_size: minimum number of points for a valid cluster.
        max_cluster_points: maximum number of points allowed per cluster
                            before splitting into subclusters.

    Returns:
        clusters: list of clusters (each a list of (x, y) world points).
    """

    if not frontier_cells:
        return []

    # --- Map scale and resolution ---
    width = og.info.width
    height = og.info.height
    res = og.info.resolution

    # Convert all frontier cells to world coordinates
    points = np.array([grid_to_world(og, c, r) for (c, r) in frontier_cells])

    # --- Dynamic epsilon selection ---
    if eps is None:
        map_width_m = width * res
        map_height_m = height * res
        map_diag = math.sqrt(map_width_m ** 2 + map_height_m ** 2)
        eps = max(2.5 * res, 0.005 * map_diag)  # scales well indoors & outdoors

    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)

    clusters = []
    for cluster_id in set(clustering.labels_):
        if cluster_id == -1:
            continue  # skip noise

        indices = np.where(clustering.labels_ == cluster_id)[0]
        cluster_points = points[indices]

        # Skip very small noise clusters
        if len(cluster_points) < min_cluster_size:
            continue

        # --- Split overly large clusters ---
        if len(cluster_points) > max_cluster_points:
            num_subclusters = int(np.ceil(len(cluster_points) / max_cluster_points))
            sorted_points = cluster_points[np.argsort(cluster_points[:, 0])]
            subclusters = np.array_split(sorted_points, num_subclusters)
            for sc in subclusters:
                clusters.append(sc.tolist())
        else:
            clusters.append(cluster_points.tolist())

    # --- Logging ---
    try:
        print(f"[FrontierCluster] eps={eps:.3f} | {len(clusters)} clusters | "
              f"avg points/cluster={np.mean([len(c) for c in clusters]):.1f}")
    except Exception:
        pass

    return clusters


def compute_centroids(clusters):
    """Compute simple mean centroids of frontier clusters."""
    centroids = []
    for cluster in clusters:
        arr = np.array(cluster)
        if arr.shape[0] == 0:
            continue
        cx, cy = np.mean(arr[:, 0]), np.mean(arr[:, 1])
        centroids.append((cx, cy))
    return centroids


def is_point_in_known_area(occupancy_grid: OccupancyGrid, x: float, y: float,
                           free_thresh: float = 0.25) -> bool:
    """
    Checks whether (x, y) lies in the known area of the occupancy grid
    (not unknown, and below free threshold).
    """
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    res = occupancy_grid.info.resolution
    origin = occupancy_grid.info.origin
    ox, oy = origin.position.x, origin.position.y

    # Convert world → grid
    col = int((x - ox) / res)
    row = int((y - oy) / res)

    if col < 0 or row < 0 or col >= width or row >= height:
        return False

    val = occupancy_grid.data[row * width + col]

    # Unknown cell
    if val < 0:
        return False

    # Known and reasonably free
    return val < int(free_thresh * 100)