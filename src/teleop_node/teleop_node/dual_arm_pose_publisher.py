import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import cv2
import mediapipe as mp
from transforms3d.euler import euler2quat
from builtin_interfaces.msg import Time

class DualArmTeleopNode(Node):
    def __init__(self):
        super().__init__('dual_arm_teleop_node')

        self.publisher_left = self.create_publisher(PoseStamped, '/left_arm/pose_goal', 10)
        self.publisher_right = self.create_publisher(PoseStamped, '/right_arm/pose_goal', 10)

        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False,
                                         max_num_hands=2,
                                         min_detection_confidence=0.5,
                                         min_tracking_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Camera frame not captured')
            return

        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks, hand_info in zip(results.multi_hand_landmarks, results.multi_handedness):
                handedness = hand_info.classification[0].label  # 'Left' or 'Right'
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]

                # Convert to pseudo-3D pose (x and y normalized from image, z is mocked)
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "world"
                pose.pose.position.x = wrist.x * 0.5  # scale to robot workspace
                pose.pose.position.y = wrist.y * 0.5
                pose.pose.position.z = 0.2  # fixed height for simplicity

                # Orientation: face downward
                qw, qx, qy, qz = euler2quat(0, 0, 0)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

                if handedness == 'Left':
                    self.publisher_left.publish(pose)
                    self.get_logger().info(f'Published LEFT hand pose: {pose.pose.position}')
                else:
                    self.publisher_right.publish(pose)
                    self.get_logger().info(f'Published RIGHT hand pose: {pose.pose.position}')

                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        cv2.imshow('Hand Tracker', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DualArmTeleopNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

