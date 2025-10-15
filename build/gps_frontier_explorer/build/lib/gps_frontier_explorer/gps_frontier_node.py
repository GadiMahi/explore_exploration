import rclpy 
from rclpy.node import Node
from gps_frontier_explorer.nav2_client import Nav2Client, TaskResult

class GPSFrontierExplorer(Node):

    def __init__(self):
        super().__init__('gps_frontier_node')

        self.nav = Nav2Client()

        #dummy goal for testing
        x, y, yaw = 2.0, 1.0, 0.0
        self.get_logger().info(f"Sending dummy goal: ({x},{y})")

        self.nav.go_to_xy(x, y, yaw)

        while not self.nav.is_task_complete():
            self.nav.spin_once(0.1)

        result = self.nav.result()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")

def main(args=None):
    rclpy.init(args=args)
    node = GPSFrontierExplorer()
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':  
    main()