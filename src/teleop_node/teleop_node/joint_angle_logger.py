import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from datetime import datetime
import os

class JointAngleLogger(Node):

    def __init__(self):
        super().__init__('joint_angle_logger')

        # Define a consistent log file path (shared for all runs)
        log_dir = os.path.expanduser('~/sereact_ws/joint_logs')
        os.makedirs(log_dir, exist_ok=True)
        self.log_file_path = os.path.join(log_dir, 'joint_angles_log.txt')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info(f"Logging joint angles to: {self.log_file_path}")

    def listener_callback(self, msg):
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        joint_info = zip(msg.name, msg.position)
        formatted = ', '.join([f'{name}: {pos:.4f}' for name, pos in joint_info])

        with open(self.log_file_path, 'a') as f:  # Append mode
            f.write(f'[{now}] {formatted}\n')

def main(args=None):
    rclpy.init(args=args)
    node = JointAngleLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

