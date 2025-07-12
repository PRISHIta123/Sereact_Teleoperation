import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import os
from datetime import datetime

class DualArmPoseListener(Node):
    def __init__(self):
        super().__init__('dual_arm_pose_listener')

        # Create logging directory
        self.log_dir = os.path.expanduser('~/sereact_ws/hand_pose_logs')
        os.makedirs(self.log_dir, exist_ok=True)

        # Create exactly two files (one per arm)
        self.left_log_path = os.path.join(self.log_dir, 'left_arm_poses.txt')
        self.right_log_path = os.path.join(self.log_dir, 'right_arm_poses.txt')

        self.left_log_file = open(self.left_log_path, 'w')
        self.right_log_file = open(self.right_log_path, 'w')

        # Write headers
        header = 'timestamp,x,y,z,qx,qy,qz,qw\n'
        self.left_log_file.write(header)
        self.right_log_file.write(header)

        # Subscriptions
        self.subscription_left = self.create_subscription(
            PoseStamped,
            '/left_arm/pose_goal',
            self.left_callback,
            10)

        self.subscription_right = self.create_subscription(
            PoseStamped,
            '/right_arm/pose_goal',
            self.right_callback,
            10)

    def left_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        timestamp = self.get_clock().now().to_msg()
        line = f'{timestamp.sec}.{timestamp.nanosec},{pos.x:.4f},{pos.y:.4f},{pos.z:.4f},{ori.x:.4f},{ori.y:.4f},{ori.z:.4f},{ori.w:.4f}\n'
        self.left_log_file.write(line)
        self.left_log_file.flush()
        self.get_logger().info(f'[LEFT] {line.strip()}')

    def right_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        timestamp = self.get_clock().now().to_msg()
        line = f'{timestamp.sec}.{timestamp.nanosec},{pos.x:.4f},{pos.y:.4f},{pos.z:.4f},{ori.x:.4f},{ori.y:.4f},{ori.z:.4f},{ori.w:.4f}\n'
        self.right_log_file.write(line)
        self.right_log_file.flush()
        self.get_logger().info(f'[RIGHT] {line.strip()}')

    def destroy_node(self):
        self.left_log_file.close()
        self.right_log_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualArmPoseListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

