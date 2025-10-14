import numpy as np
from sklearn.cluster import DBSCAN
from gps_frontier_explorer.frontier_detection import grid_to_world

def cluster_frontiers(frontier_cells, og, eps: float = None, min_samples: int = 5, min_cluster_size: int = 5):
    if not frontier_cells:
        return []
    
    res = og.info.resolution 

    if eps is None:
        eps = 2.5 * res
    points = np.array([grid_to_world(og, c, r) for (c,r) in frontier_cells])

    clustering = DBSCAN(eps = eps, min_samples=min_samples).fit(points)

    clusters = []
    for cluster_id in set(clustering.labels_):
        if cluster_id == -1:
            continue
        indices = np.where(clustering.labels_ == cluster_id)[0]
        cluster_points = points[indices]

        if len(cluster_points) <5:
            continue
        clusters.append(cluster_points.tolist())
    return clusters

def compute_centroids(clusters):
    centroids = []
    for cluster in clusters:
        arr = np.array(cluster)
        cx, cy = np.mean(arr[:, 0]), np.mean(arr[:, 1])
        centroids.append((cx, cy))
    return centroids