from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math
import rclpy
import tf_transformations

def yaw_to_quat(yaw: float):
    half = yaw/2.0
    return [0.0, 0.0, math.sin(half), math.cos(half)]

def create_pose_stamped(navigator, position_x, position_y, rotation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        return goal_pose

class Nav2Client:

    def __init__(self):
        self.navigator = BasicNavigator()

        initial_pose = create_pose_stamped(self.navigator, -2.0, -0.5, 0)
        self.navigator.setInitialPose(initial_pose)
        #wait until Nav2 is active (bringup must be running )
        self.navigator.waitUntilNav2Active()
    
        
        

    def go_to_xy(self, x: float, y: float, yaw: float = 0.0, frame_id: str = "map"):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.navigator.goToPose(pose)

    def is_task_complete(self) -> bool:
        return self.navigator.isTaskComplete()

    def result(self):
        return self.navigator.getResult()
    
    def cancel(self):
        self.navigator.cancelTask()

    def spin_once (self, timeout_sec: float = 0.1):
        rclpy.spin_once(self.navigator.__node, timeout_sec=0.1)

    

    

        