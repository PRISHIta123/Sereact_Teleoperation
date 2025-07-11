import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateRepublisher(Node):
    def __init__(self):
        super().__init__('joint_state_republisher')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(
            JointState,
            '/current_joint_angles',
            10)
        
        self.get_logger().info('Republishing joint angles from /joint_states → /current_joint_angles')

    def listener_callback(self, msg: JointState):
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
