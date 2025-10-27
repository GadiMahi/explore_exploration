# exploring_exploration/test_frontier_node.py

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from gps_frontier_explorer.frontier_detection import detect_frontier_cells, grid_to_world
from visualization_msgs.msg import Marker, MarkerArray
from gps_frontier_explorer.nav2_client import Nav2Client
from gps_frontier_explorer.frontier_utils import cluster_frontiers, compute_centroids, is_point_in_known_area
from gps_frontier_explorer.frontier_selection import select_best_centroid
import tf2_ros
from geometry_msgs.msg import TransformStamped
from typing import Optional, Tuple, Sequence, Iterable
from nav2_simple_commander.robot_navigator import TaskResult
import tf_transformations


class TestFrontierNode(Node):
    def __init__(self):
        super().__init__('test_frontier_node')
        
        # Parameters
        self.goal_x, self.goal_y = 2.5, -4.0
        self.sent_final_goal = False
        self.goal_active = False
        self.visited_frontiers = []
        self.last_goal = None
        self.skip_radius = 0.30
        self.tie_threshold = 0.10
        self.visited_limit = 30
        self.rotation_in_progress = False
    
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        
        # Subscribers and Publishers
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'frontier_marker', 10)

        # Nav2 Commander Client
        self.navigator = Nav2Client()
        
        # Periodic Nav2 goal status checker
        self.timer = self.create_timer(0.5, self.check_nav_status)

        self.declare_parameter("debug_visualization", False)
        self.debug_visualization = self.get_parameter("debug_visualization").get_parameter_value().bool_value
    
    def get_robot_xy(self) -> Optional[tuple]:
        try:
            tf:TransformStamped = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def map_callback(self, msg):
        """Triggered when the /map topic updates(from SLAM)."""

        self.latest_map = msg  # store for other methods

        # Don't re-run logic while navigating
        if self.goal_active:
            return 
        
        # visualize 
        if self.debug_visualization:
            self.visualize_processed_cells(msg)

        # if final goal is already inside known map -> send it once
        if is_point_in_known_area(msg, self.goal_x, self.goal_y):
            if not self.sent_final_goal:
                self.get_logger().info("Goal is inside known map - sending final goal to Nav2.")
                self.navigator.go_to_xy(self.goal_x, self.goal_y, 0.0, frame_id="map")
                self.goal_active = True
                self.sent_final_goal = True
                self.marker_pub.publish(self._delete_all_array())
            return
        
        # Otherwise:detect frontiers -> cluster -> pick best centroid
        cells = detect_frontier_cells(msg)
        self.get_logger().info(f"Detected {len(cells)} frontier cells")

        clusters = cluster_frontiers(cells, msg)
        centroids = compute_centroids(clusters)
        self.get_logger().info(f"Found {len(clusters)} clusters, {len(centroids)} centroids")

        markers = MarkerArray()
        markers.markers.append(self._delete_all())
        for i, (cx,cy) in enumerate(centroids):

            m = Marker()
            m.header.frame_id = "map" 
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = f"frontier_centroids"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.05

            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.r, m.color.g, m.color.b, m.color.a= 0.0, 1.0, 0.0, 1.0

            markers.markers.append(m)

        # Select the best frontier centroid toward goal
        robot_xy = self.get_robot_xy()
        candidates = select_best_centroid(
            centroids,
            self.goal_x,
            self.goal_y,
            last_goal=self.last_goal,
            visited=self.visited_frontiers,
            skip_radius=self.skip_radius,
            tie_threshold=self.tie_threshold,
            robot_xy=robot_xy
        )
        if not candidates:
            self.get_logger().warning("No valid frontier centroids found after filtering.")
            return
        chosen_frontier = None
        for cx, cy, d_goal, d_robot in candidates:
            if not self.last_goal or math.hypot(cx - self.last_goal[0], cy - self.last_goal[1]) > self.skip_radius:
                chosen_frontier = (cx, cy, d_goal, d_robot)
                break  # take the best one

        if not chosen_frontier:
            cx, cy = candidates[0][0], candidates[0][1]
        else:
            cx, cy = chosen_frontier[0], chosen_frontier[1] 
        
        yaw = math.atan2(self.goal_y - cy, self.goal_x - cx)
        self.get_logger().info(f"Selected frontier at ({cx:.2f}, {cy:.2f}) -> yaw {math.degrees(yaw):.1f}°")


        self.stage = 'IDLE'
        self.pending_frontier = None
        self.pending_backtrack = None

        bx, by = self._compute_backtrack_point(cx, cy)
        self.pending_frontier = (cx, cy)
        if is_point_in_known_area(msg, bx, by):
            target = (bx,by)
            self.stage = "BACKTRACK"
        else:
            target = (cx,cy)
            self.stage = "FRONTIER"

        

        # Highlight chosen centroid
        best_marker = Marker()
        best_marker.header.frame_id = "map"
        best_marker.header.stamp = self.get_clock().now().to_msg()
        best_marker.ns = "best_centroid"
        best_marker.id = 0
        best_marker.type = Marker.CUBE
        best_marker.action = Marker.ADD
        best_marker.pose.position.x = cx
        best_marker.pose.position.y = cy
        best_marker.pose.position.z = 0.2
        best_marker.scale.x = best_marker.scale.y = best_marker.scale.z = 0.35
        best_marker.color.r, best_marker.color.g, best_marker.color.b, best_marker.color.a = 1.0, 0.0, 0.0, 1.0

        markers.markers.append(best_marker)


        yaw = math.atan2(self.goal_y - cy, self.goal_x - cx)
        self.navigator.go_to_xy(target[0], target[1], 0.00)
        self.goal_active = True

        # Draw final global goal (blue sphere)
        goal_marker = Marker()
        goal_marker.header.frame_id = "map"
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = "goal"
        goal_marker.id = 9999
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = self.goal_x
        goal_marker.pose.position.y = self.goal_y
        goal_marker.pose.position.z = 0.1
        goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.3
        goal_marker.color.r, goal_marker.color.g, goal_marker.color.b, goal_marker.color.a = 0.0, 0.0, 1.0, 1.0
        markers.markers.append(goal_marker)

        self.marker_pub.publish(markers)


    
    def visualize_processed_cells(self, og: OccupancyGrid):
        width = og.info.width
        height = og.info.height
        res = og.info.resolution
        ox, oy = og.info.origin.position.x, og.info.origin.position.y

        markers = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        id_counter = 0
        for row in range(height):
            for col in range(width):
                wx = ox + (col + 0.5) * res
                wy = oy + (row + 0.5) * res

                m=Marker()
                m.header.frame_id = "map"
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "processed_cells"
                m.id = id_counter
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x = wx
                m.pose.position.y = wy
                m.pose.position.z = 0.01
                m.scale.x = m.scale.y = res
                m.scale.z = 0.01
                m.color.r, m.color.g, m.color.b, m.color.a = 0.5, 0.0, 0.5, 0.3

                markers.markers.append(m)
                id_counter+=1
        self.marker_pub.publish(markers)

    def check_nav_status(self):
        """Periodically check Nav2 progress and handle multi-stage navigation (backtrack → frontier)."""
        self.navigator.spin_once(0.05)

        if not self.goal_active:
            return  # Nothing to check if no goal in progress

        done = self.navigator.is_task_complete()
        if not done:
            return

        # --- Get Nav2 result ---
        result = self.navigator.result()
        self.get_logger().info(f"[nav] Goal completed with result: {result}")

        # Normalize result to integer code
        try:
            if result == TaskResult.SUCCEEDED:
                code = 0
            elif result == TaskResult.CANCELED:
                code = 1
            elif result == TaskResult.FAILED:
                code = 2
            else:
                code = -1
        except Exception as e:
            code = int(result) if isinstance(result, int) else -1
            self.get_logger().warn(f"Error interpreting Nav2 result: {e}")

        # --- Success case ---
        if code == 0:
            if getattr(self, "stage", "IDLE") == "BACKTRACK" and hasattr(self, "pending_frontier"):
                # Completed backtrack leg → now go to the actual frontier
                cx, cy = self.pending_frontier
                yaw = math.atan2(self.goal_y - cy, self.goal_x - cx)
                self.get_logger().info(f"Reached backtrack point. Proceeding to frontier ({cx:.2f}, {cy:.2f})...")
                self.navigator.go_to_xy(cx, cy, yaw)
                self.stage = "FRONTIER"
                return

            elif getattr(self, "stage", "IDLE") == "FRONTIER":
                # Fully reached the frontier — now mark as visited
                if hasattr(self, "pending_frontier") and self.pending_frontier:
                    cx, cy = self.pending_frontier
                    self.last_goal = (cx, cy)
                    self.visited_frontiers.append((cx, cy))
                    if len(self.visited_frontiers) > self.visited_limit:
                        self.visited_frontiers.pop(0)
                self.get_logger().info("Frontier reached successfully.")
                self.stage = "IDLE"
                self.goal_active = False
                self.pending_frontier = None
                return

            else:
                # Normal single-stage goal (no backtrack)
                self.get_logger().info("Goal reached successfully (no backtrack stage).")
                self.goal_active = False
                self.stage = "IDLE"

        # --- Canceled or failed cases ---
        elif code == 1:
            self.get_logger().warn("Goal was canceled — trying next frontier.")
            self.stage = "IDLE"
            self._handle_failed_goal()

        elif code == 2:
            self.get_logger().warn("Goal unreachable — trying next frontier.")
            self.stage = "IDLE"
            self._handle_failed_goal()

        # --- Final goal case ---
        if self.sent_final_goal and code == 0:
            self.get_logger().info("FINAL GOAL REACHED — EXPLORATION COMPLETE.")

    
    def perform_goal_scan(self):
        """Rotate gently only if map didn't expand and goal is still unknown."""
        if not self.last_goal or not hasattr(self, "latest_map"):
            return

        gx, gy = self.goal_x, self.goal_y
        rx, ry = self.last_goal

        # Check if goal is now visible
        if is_point_in_known_area(self.latest_map, gx, gy):
            self.get_logger().info("Goal already visible — no rotation needed.")
            return

        # Measure known cell count change
        known_now = sum(v >= 0 for v in self.latest_map.data)
        delta = known_now - getattr(self, "last_known_count", 0)
        self.last_known_count = known_now

        if delta > 500:  # enough map expansion
            self.get_logger().info("Map expanded — skipping rotation.")
            return

        self.get_logger().warning("Map stagnant, performing gentle goal-facing scan...")

        yaw_to_goal = math.atan2(gy - ry, gx - rx)

        # Perform a short ±15° sweep
        for delta_yaw in [-0.25, 0.0, 0.25]:  # radians (~15°)
            yaw = yaw_to_goal + delta_yaw
            self.navigator.go_to_xy(rx, ry, yaw)
            self.get_logger().info(f"Rotating slightly to yaw {math.degrees(yaw):.1f}°")
            self.navigator.spin_once(0.5)

        self.get_logger().info("Scan complete — waiting for new frontiers...")

    def _compute_backtrack_point(self, fx, fy, distance=0.4):
        """
        Compute a point slightly behind the frontier (away from the goal).
        Returns (bx, by).
        """
        dx = fx - self.goal_x
        dy = fy - self.goal_y
        length = math.hypot(dx, dy)
        if length == 0:
            return fx, fy  # already at goal
        
        ux = dx / length
        uy = dy / length

        bx = fx + ux * distance
        by = fy + uy * distance 
        return bx, by

    def _handle_failed_goal(self):
        """Mark failed frontier and retry next best centroid."""
        if self.last_goal:
            self.visited_frontiers.append(self.last_goal)
            if len(self.visited_frontiers) > self.visited_limit:
                self.visited_frontiers.pop(0)

        self.get_logger().info("Selecting next reachable frontier...")
        self.goal_active = False  # allow map_callback to trigger again

    def _rotate_toward_goal(self):
        """Rotate rover in place to face the final goal, then wait for SLAM update."""
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
            rot = tf.transform.rotation

            # Convert current orientation quaternion to yaw
            _, _, current_yaw = tf_transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

            # Compute direction to goal
            dx = self.goal_x - rx
            dy = self.goal_y - ry
            goal_yaw = math.atan2(dy, dx)

            # Compute yaw difference (normalized to [-π, π])
            yaw_diff = math.atan2(math.sin(goal_yaw - current_yaw), math.cos(goal_yaw - current_yaw))

            self.get_logger().info(
                f"Current yaw: {math.degrees(current_yaw):.1f}°, "
                f"Goal yaw: {math.degrees(goal_yaw):.1f}°, "
                f"Need to turn: {math.degrees(yaw_diff):.1f}°"
            )

            # Command rotation toward desired yaw
            target_yaw = current_yaw + yaw_diff
            self.navigator.go_to_xy(rx, ry, target_yaw)

            # Wait for rotation completion
            while not self.navigator.is_task_complete():
                self.navigator.spin_once(0.1)

            self.get_logger().info("Alignment complete. Waiting for SLAM map to update...")
            time.sleep(3.0)
            self.goal_active = False

        except Exception as e:
            self.get_logger().warning(f"Rotation toward goal failed: {e}")

    def _delete_all(self) -> Marker:
        m = Marker()
        m.action = Marker.DELETEALL
        return m

    def _delete_all_array(self) -> MarkerArray:
        arr = MarkerArray()
        arr.markers.append(self._delete_all())
        return arr




        


def main(args=None):
    rclpy.init(args=args)
    node = TestFrontierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
