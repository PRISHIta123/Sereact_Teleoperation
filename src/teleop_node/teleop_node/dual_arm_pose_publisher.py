import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import cv2
import mediapipe as mp
from transforms3d.euler import euler2quat

class DualArmPosePublisher(Node):
    def __init__(self):
        super().__init__('dual_arm_teleop_node')

        self.publisher_left_pose = self.create_publisher(PoseStamped, '/left_arm/pose_goal', 10)
        self.publisher_right_pose = self.create_publisher(PoseStamped, '/right_arm/pose_goal', 10)
        self.publisher_left_grip = self.create_publisher(Bool, '/left_arm/gripper_state', 10)
        self.publisher_right_grip = self.create_publisher(Bool, '/right_arm/gripper_state', 10)

        self.cap = cv2.VideoCapture(0)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils

        self.timer = self.create_timer(0.1, self.timer_callback)

    def is_hand_closed(self, landmarks):
        # Count how many fingers are folded (tip below PIP in y-axis for right hand)
        folded_count = 0

        finger_tips = [
             self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
             self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
             self.mp_hands.HandLandmark.RING_FINGER_TIP,
             self.mp_hands.HandLandmark.PINKY_TIP
        ]

        finger_pips = [
             self.mp_hands.HandLandmark.INDEX_FINGER_PIP,
             self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
             self.mp_hands.HandLandmark.RING_FINGER_PIP,
             self.mp_hands.HandLandmark.PINKY_PIP
        ]

        for tip, pip in zip(finger_tips, finger_pips):
            if landmarks[tip].y > landmarks[pip].y:  # y increases downward in image
                folded_count += 1

        # Consider thumb separately: folded if tip is left of IP for right hand
        thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = landmarks[self.mp_hands.HandLandmark.THUMB_IP]
        thumb_folded = abs(thumb_tip.x - thumb_ip.x) < 0.02

        return folded_count >= 3 and thumb_folded


    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Camera frame not captured")
            return

        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks, hand_info in zip(results.multi_hand_landmarks, results.multi_handedness):
                handedness = hand_info.classification[0].label  # 'Left' or 'Right'
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]

                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "world"
                pose.pose.position.x = wrist.x * 0.5
                pose.pose.position.y = wrist.y * 0.5
                pose.pose.position.z = 0.2

                qw, qx, qy, qz = euler2quat(0, 0, 0)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

                is_closed = Bool()
                is_closed.data = self.is_hand_closed(hand_landmarks.landmark)

                if handedness == 'Left':
                    self.publisher_left_pose.publish(pose)
                    self.publisher_left_grip.publish(is_closed)
                    self.get_logger().info(f'[LEFT] Pose: {pose.pose.position}, Gripper Closed: {is_closed.data}')
                else:
                    self.publisher_right_pose.publish(pose)
                    self.publisher_right_grip.publish(is_closed)
                    self.get_logger().info(f'[RIGHT] Pose: {pose.pose.position}, Gripper Closed: {is_closed.data}')

                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        cv2.imshow('Hand Tracker', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DualArmPosePublisher()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

