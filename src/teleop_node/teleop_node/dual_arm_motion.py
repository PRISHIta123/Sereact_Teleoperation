#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time

class DualArmPublisher(Node):
    def __init__(self):
        super().__init__('dual_arm_motion')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.joint_names = [
            # Left arm
            'left_panda_joint1', 'left_panda_joint2', 'left_panda_joint3',
            'left_panda_joint4', 'left_panda_joint5', 'left_panda_joint6',
            'left_panda_joint7',
            # Right arm
            'right_panda_joint1', 'right_panda_joint2', 'right_panda_joint3',
            'right_panda_joint4', 'right_panda_joint5', 'right_panda_joint6',
            'right_panda_joint7',
        ]

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        amplitude = 0.4

        # Simple oscillating motion
        positions = [amplitude * math.sin(t + i) for i in range(14)]

        msg.name = self.joint_names
        msg.position = positions
        self.publisher_.publish(msg)
        self.get_logger().info("Published joint positions")

def main(args=None):
    rclpy.init(args=args)
    node = DualArmPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

