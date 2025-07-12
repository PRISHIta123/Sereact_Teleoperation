import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import os

class DualArmPoseListener(Node):
    def __init__(self):
        super().__init__('dual_arm_pose_listener')

        log_dir = os.path.expanduser('~/sereact_ws/hand_pose_logs')
        os.makedirs(log_dir, exist_ok=True)

        self.left_log = open(os.path.join(log_dir, 'left_arm_poses.txt'), 'w')
        self.right_log = open(os.path.join(log_dir, 'right_arm_poses.txt'), 'w')

        header = 'timestamp,x,y,z,qx,qy,qz,qw,gripper_closed\n'
        self.left_log.write(header)
        self.right_log.write(header)

        self.left_gripper_state = False
        self.right_gripper_state = False

        self.create_subscription(PoseStamped, '/left_arm/pose_goal', self.left_pose_cb, 10)
        self.create_subscription(PoseStamped, '/right_arm/pose_goal', self.right_pose_cb, 10)
        self.create_subscription(Bool, '/left_arm/gripper_state', self.left_grip_cb, 10)
        self.create_subscription(Bool, '/right_arm/gripper_state', self.right_grip_cb, 10)

    def left_grip_cb(self, msg):
        self.left_gripper_state = msg.data

    def right_grip_cb(self, msg):
        self.right_gripper_state = msg.data

    def left_pose_cb(self, msg):
        self.log_pose(msg, self.left_log, self.left_gripper_state, 'LEFT')

    def right_pose_cb(self, msg):
        self.log_pose(msg, self.right_log, self.right_gripper_state, 'RIGHT')

    def log_pose(self, msg, file, grip_state, label):
        p = msg.pose.position
        o = msg.pose.orientation
        t = msg.header.stamp
        line = f'{t.sec}.{t.nanosec},{p.x:.4f},{p.y:.4f},{p.z:.4f},{o.x:.4f},{o.y:.4f},{o.z:.4f},{o.w:.4f},{int(grip_state)}\n'
        file.write(line)
        file.flush()
        self.get_logger().info(f'[{label}] {line.strip()}')

    def destroy_node(self):
        self.left_log.close()
        self.right_log.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualArmPoseListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

