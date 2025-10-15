import math
import numpy as np
from sklearn.cluster import DBSCAN
from gps_frontier_explorer.frontier_detection import grid_to_world
from nav_msgs.msg import OccupancyGrid

def cluster_frontiers(frontier_cells, og, eps: float = None, min_samples: int = 3, min_cluster_size: int = 3):
    if not frontier_cells:
        return []
    
    
    width = og.info.width
    height = og.info.height
    res = og.info.resolution 

    map_width_m = width * res
    map_height_m = height * res
    map_diag = math.sqrt(map_width_m**2 + map_height_m**2)
    
    if eps is None:
        eps = max(2*res, 0.005 * map_diag)
    points = np.array([grid_to_world(og, c, r) for (c,r) in frontier_cells])

    clustering = DBSCAN(eps = eps, min_samples=min_samples).fit(points)

    clusters = []
    for cluster_id in set(clustering.labels_):
        if cluster_id == -1:
            continue
        indices = np.where(clustering.labels_ == cluster_id)[0]
        cluster_points = points[indices]

        if len(cluster_points) < 1:
            continue
        clusters.append(cluster_points.tolist())

    for i, c in enumerate(clusters):
        print(f"Cluster {i}: {len(c)} points")

    return clusters

def compute_centroids(clusters):
    centroids = []
    for cluster in clusters:
        arr = np.array(cluster)
        cx, cy = np.mean(arr[:, 0]), np.mean(arr[:, 1])
        centroids.append((cx, cy))
    return centroids


def is_point_in_known_area(occupancy_grid: OccupancyGrid, x: float, y: float, free_thresh: float = 0.25) -> bool:
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    res = occupancy_grid.info.resolution
    origin = occupancy_grid.info.origin
    ox, oy = origin.position.x, origin.position.y


    col = int((x - ox) / res)
    row = int((y - oy) / res)

    if col<0 or row<0 or col>=width or row>=height:
        return False
    
    val = occupancy_grid.data[row*width + col]

    if val<0:
        return False
    
    return val < int(free_thresh * 100)
