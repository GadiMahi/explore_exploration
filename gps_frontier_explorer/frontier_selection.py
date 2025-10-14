import math

def select_best_centroid(centroids, goal_x, goal_y, robot_x = None, robot_y = None, tie_threshold = 0.5):
    
    if not centroids:
        return None
    
    best = None
    min_dist = float("inf")

    for (cx, cy) in centroids:
        d_goal = math.hypot(cx - goal_x, cy - goal_y)

        if (d_goal < min_dist - tie_threshold):
            best = (cx, cy, d_goal)
            min_dist = d_goal

        elif abs(d_goal - min_dist) <= tie_threshold:

            if robot_x is not None and robot_y is not None and best is not None:
                d_robot = math.hypot(cx-robot_x, cy - robot_y)
                d_robot_best = math.hypot(best[0] - robot_x, best[1] -  robot_y)
                if d_robot < d_robot_best:
                    best = (cx, cy, d_goal)

    return best