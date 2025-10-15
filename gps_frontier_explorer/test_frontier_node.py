# exploring_exploration/test_frontier_node.py

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


class TestFrontierNode(Node):
    def __init__(self):
        super().__init__('test_frontier_node')
        
        # Parameters
        self.goal_x, self.goal_y = 3.0, 1.0
        self.sent_final_goal = False
        self.goal_active = False
        self.visited_frontiers = []
        self.last_goal = None
        self.skip_radius = 0.30
        self.tie_threshold = 0.10
        self.visited_limit = 30
    
        
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
        best = select_best_centroid(centroids, self.goal_x, self.goal_y, last_goal = self.last_goal, visited = self.visited_frontiers, skip_radius = self.skip_radius, tie_threshold = self.tie_threshold, robot_xy = robot_xy)
        if best:
            cx, cy, d_goal, d_robot = best

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

            self.get_logger().info(f"Sending Nav2 goal to centroid at ({cx:.2f}, {cy:.2f})")
            
            # Send goal to Nav2
            self.navigator.go_to_xy(cx, cy, 0.0)
            self.goal_active = True
            self.last_goal = (cx, cy)
            self.visited_frontiers.append(self.last_goal)
            if len(self.visited_frontiers) > self.visited_limit:
                self.visited_frontiers.pop(0)

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
        """ Called periodically to monitor Nav2 Progress"""
        self.navigator.spin_once(0.05)

        if self.goal_active and self.navigator.is_task_complete():
            result = self.navigator.result()
            self.get_logger().info(f"Goal finished with result: {result}")
            self.goal_active = False

            if self.sent_final_goal:
                self.get_logger().info("FINAL GOAL REACHED - EXPLORATION COMPLETE")

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
