# exploring_exploration/test_frontier_node.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from gps_frontier_explorer.frontier_detection import detect_frontier_cells, grid_to_world
from visualization_msgs.msg import Marker, MarkerArray
from gps_frontier_explorer.nav2_client import Nav2Client
from gps_frontier_explorer.frontier_utils import cluster_frontiers, compute_centroids
from gps_frontier_explorer.frontier_selection import select_best_centroid

class TestFrontierNode(Node):
    def __init__(self):
        super().__init__('test_frontier_node')
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.marker_pub = self.create_publisher(MarkerArray, 'frontier_marker', 10)
        self.navigator = Nav2Client()
        self.goal_active = False

        self.timer = self.create_timer(0.5, self.check_nav_status)

    def map_callback(self, msg):

        if self.goal_active:
            return 
        self.visualize_processed_cells(msg)
        cells = detect_frontier_cells(msg)
        self.get_logger().info(f"Detected {len(cells)} frontier cells")

        clusters = cluster_frontiers(cells, msg)
        centroids = compute_centroids(clusters)
        self.get_logger().info(f"Found {len(clusters)} clusters, {len(centroids)} centroids")

        markers = MarkerArray()
        markers.markers.clear()
        for i, (cx,cy) in enumerate(centroids):

            m = Marker()
            m.header.frame_id = "map" 
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = f"frontier_{self.get_clock().now().nanoseconds}"
            m.id = m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.05

            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.r, m.color.g, m.color.b, m.color.a= 0.0, 1.0, 0.0, 1.0

            markers.markers.append(m)

        goal_x, goal_y = 3.0, 2.0
        best = select_best_centroid(centroids, goal_x, goal_y)

        if best:
            cx, cy, dist = best

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
            #self.navigator.set_initial_pose()
            self.navigator.go_to_xy(cx, cy, 0.0)
            self.goal_active = True


        goal_marker = Marker()
        goal_marker.header.frame_id = "map"
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = "goal"
        goal_marker.id = 9999
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = goal_x
        goal_marker.pose.position.y = goal_y
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
        if self.goal_active and self.navigator.is_task_complete():
            result = self.navigator.result()
            self.get_logger().info(f"Goal finished with result: {result}")
            self.goal_active = False




        


def main(args=None):
    rclpy.init(args=args)
    node = TestFrontierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
