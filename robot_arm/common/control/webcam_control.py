"""
Webcam Control for Robot Arm with Full 6-DOF Control
This includes gesture recognition, safety features, and intuitive control mapping.
"""

import mediapipe as mp
from robot_arm.common.arm import RobotArm
from robot_arm.common.utils import setup_logging, parse_args
import cv2
import math
import time

logger = setup_logging()


class WebcamControl:
    def __init__(self):
        self.arm = RobotArm()
        self.arm.connect()

        # MediaPipe setup
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.mp_hands = mp.solutions.hands

        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        # Control parameters
        self.last_positions = {}
        self.reference_pose = None
        self.smoothing_factor = 0.8
        self.movement_threshold = 5
        self.pose_control_active = False
        self.hand_control_active = False
        self.safety_bounds_active = False

        # Performance tracking
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0

    def calculate_motor_positions(self, pose_landmarks, hand_landmarks=None):
        """Motor position calculation."""
        motor_updates = {}

        # Get key pose landmarks
        left_shoulder = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = pose_landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        right_elbow = pose_landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW]
        right_wrist = pose_landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST]
        left_hip = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_HIP]
        right_hip = pose_landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP]

        # Motor 1: Base rotation (body orientation + right arm horizontal position)
        if self.pose_control_active and all(lm.visibility > 0.5 for lm in [left_hip, right_hip, left_shoulder, right_shoulder]):
            # Get difference between left and right hip and shoulders
            shoulder_line = right_shoulder.x - left_shoulder.x
            hip_line = right_hip.x - left_hip.x
            # Weighted combination
            base_rotation = (shoulder_line * 0.3 + hip_line * 0.7) * 100

            motor1_pos = self._map_to_motor_range(1, base_rotation, -50, 50)
            if motor1_pos is not None:
                motor_updates[1] = motor1_pos

        # Motor 2: Shoulder angle (angle between chest and arm)
        if self.pose_control_active and all(lm.visibility > 0.5 for lm in [left_shoulder, right_shoulder, right_elbow]):
            # Calculate angle between chest line and upper arm
            chest_vector = [right_shoulder.x - left_shoulder.x,
                            right_shoulder.y - left_shoulder.y]
            arm_vector = [right_elbow.x - right_shoulder.x,
                          right_elbow.y - right_shoulder.y]

            # Calculate angle between chest and arm
            chest_angle = math.atan2(chest_vector[1], chest_vector[0])
            arm_angle = math.atan2(arm_vector[1], arm_vector[0])
            shoulder_angle = math.degrees(arm_angle - chest_angle)

            # Normalize to -180 to 180
            shoulder_angle = ((shoulder_angle + 180) % 360) - 180

            motor2_pos = self._map_to_motor_range(
                2, shoulder_angle, -90, 90)
            if motor2_pos is not None:
                motor_updates[2] = motor2_pos

        # Motor 3: Elbow (bend angle)
        if self.pose_control_active and all(lm.visibility > 0.5 for lm in [right_shoulder, right_elbow, right_wrist]):
            elbow_angle = self._calculate_angle(
                right_shoulder, right_elbow, right_wrist)
            motor3_pos = self._map_to_motor_range(3, elbow_angle, 20, 160)
            if motor3_pos is not None:
                motor_updates[3] = motor3_pos

        # Motor 4: Hand pitch (vertical hand tilt)
        if hand_landmarks and self.hand_control_active:
            # Use hand vertical orientation for pitch control
            pinky_finger = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]
            wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]

            # Calculate pitch angle based on vertical position difference
            pitch_angle = math.degrees(math.atan2(
                wrist.y - pinky_finger.y,
                abs(pinky_finger.x - wrist.x) + 0.001  # Avoid division by zero
            ))
            motor4_pos = self._map_to_motor_range(4, pitch_angle, -60, 60)
            if motor4_pos is not None:
                motor_updates[4] = motor4_pos

        # Motor 5: Forearm/wrist rotation
        if hand_landmarks and self.hand_control_active:
            # Use hand horizontal orientation for forearm rotation
            thumb = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
            pinky = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]

            # Calculate rotation angle based on thumb-pinky line
            rotation_angle = math.degrees(math.atan2(
                pinky.y - thumb.y, pinky.x - thumb.x))
            motor5_pos = self._map_to_motor_range(5, rotation_angle, -90, 90)
            if motor5_pos is not None:
                motor_updates[5] = motor5_pos

        elif self.pose_control_active and all(lm.visibility > 0.5 for lm in [right_elbow, right_wrist]):
            # Fallback to wrist twist using elbow-wrist vector
            wrist_twist = math.degrees(math.atan2(
                right_wrist.x - right_elbow.x,
                right_wrist.y - right_elbow.y
            ))
            motor5_pos = self._map_to_motor_range(5, wrist_twist, -45, 45)
            if motor5_pos is not None:
                motor_updates[5] = motor5_pos

        # Motor 6: Gripper (thumb to pointer distance)
        if hand_landmarks and self.hand_control_active:
            # Use distance between thumb and index finger for gripper control
            thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
            index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

            # Calculate thumb-index distance
            thumb_index_distance = self._distance(thumb_tip, index_tip)

            motor6_pos = self._map_to_motor_range(
                6, thumb_index_distance, 0.02, 0.15, invert=True)
            if motor6_pos is not None:
                motor_updates[6] = motor6_pos

        return motor_updates

    def _distance(self, point1, point2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

    def _calculate_angle(self, point1, point2, point3):
        """Calculate angle at point2 formed by point1-point2-point3."""
        v1 = [point1.x - point2.x, point1.y - point2.y]
        v2 = [point3.x - point2.x, point3.y - point2.y]

        dot_product = v1[0] * v2[0] + v1[1] * v2[1]
        mag1 = math.sqrt(v1[0]**2 + v1[1]**2)
        mag2 = math.sqrt(v2[0]**2 + v2[1]**2)

        if mag1 == 0 or mag2 == 0:
            return 0

        cos_angle = max(min(dot_product / (mag1 * mag2), 1.0), -1.0)
        return math.degrees(math.acos(cos_angle))

    def _map_to_motor_range(self, motor_id, value, min_input, max_input, invert=False):
        """Map input value to motor position range with enhanced smoothing."""
        if motor_id not in self.arm.task_handler.motor_controller.limits:
            return None

        min_limit, max_limit = self.arm.task_handler.motor_controller.limits[motor_id]

        # Apply safety bounds
        if self.safety_bounds_active:
            safe_range = (max_limit - min_limit) * \
                0.8  # Use 80% of range for safety
            center = (min_limit + max_limit) // 2
            min_limit = int(center - safe_range // 2)
            max_limit = int(center + safe_range // 2)

        # Clamp input value
        value = max(min_input, min(max_input, value))

        # Normalize to 0-1
        normalized = (value - min_input) / (max_input - min_input)

        if invert:
            normalized = 1.0 - normalized

        # Map to motor range
        motor_pos = int(min_limit + normalized * (max_limit - min_limit))

        # Enhanced smoothing
        if motor_id in self.last_positions:
            diff = abs(motor_pos - self.last_positions[motor_id])
            # Adaptive smoothing based on movement size
            adaptive_smoothing = self.smoothing_factor if diff < 50 else 0.5
            smoothed_pos = (adaptive_smoothing * self.last_positions[motor_id] +
                            (1 - adaptive_smoothing) * motor_pos)
            motor_pos = int(smoothed_pos)

        return motor_pos

    def draw_ui(self, frame, motor_updates, pose_detected, hand_detected):
        """Draw comprehensive UI with status and controls."""
        h, w = frame.shape[:2]

        # Control panel background - larger for 720p
        cv2.rectangle(frame, (10, 10), (500, 250), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, 10), (500, 250), (255, 255, 255), 2)

        # Title - larger font for 720p
        cv2.putText(frame, "Robot Arm Control", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

        # Status indicators - adjusted spacing for 720p
        status_y = 75
        pose_color = (0, 255, 0) if pose_detected else (0, 0, 255)
        hand_color = (0, 255, 0) if hand_detected else (0, 0, 255)
        pose_control_color = (
            0, 255, 0) if self.pose_control_active else (255, 0, 0)
        hand_control_color = (
            0, 255, 0) if self.hand_control_active else (255, 0, 0)

        cv2.putText(frame, f"Pose: {'OK' if pose_detected else 'NO'}", (20, status_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, pose_color, 1)
        cv2.putText(frame, f"Hand: {'OK' if hand_detected else 'NO'}", (130, status_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, hand_color, 1)
        cv2.putText(frame, f"Pose Ctrl: {'ON' if self.pose_control_active else 'OFF'}", (240, status_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, pose_control_color, 1)
        cv2.putText(frame, f"Hand Ctrl: {'ON' if self.hand_control_active else 'OFF'}", (380, status_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, hand_control_color, 1)

        # Motor positions - adjusted spacing
        motor_y = 110
        for motor_id, position in motor_updates.items():
            cv2.putText(frame, f"M{motor_id}: {position}", (20, motor_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            motor_y += 20

        # Instructions panel - positioned for 720p
        instructions_x = w - 450
        cv2.rectangle(frame, (instructions_x, 10),
                      (w - 10, 350), (0, 0, 0), -1)
        cv2.rectangle(frame, (instructions_x, 10),
                      (w - 10, 350), (255, 255, 255), 2)

        instructions = [
            "CONTROLS:",
            "'y' = Toggle pose control",
            "'t' = Toggle hand control",
            "",
            "MOTOR MAPPING:",
            "1-3: Body pose (y key)",
            "4-6: Hand gestures (t key)",
            "",
            "POSE MOTORS:",
            "1: Body lean + arm position",
            "2: Arm height",
            "3: Elbow bend",
            "",
            "HAND MOTORS:",
            "4: Hand pitch (up/down tilt)",
            "5: Forearm/wrist rotation",
            "6: Thumb-index distance",
            "",
            "KEYS:",
            "'h' = Home position",
            "'s' = Toggle safety bounds",
            "'q' = Quit"
        ]

        for i, instruction in enumerate(instructions):
            color = (0, 255, 255) if instruction.startswith(
                ('CONTROLS:', 'MOTOR MAPPING:', 'KEYS:')) else (255, 255, 255)
            cv2.putText(frame, instruction, (instructions_x + 15, 40 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # FPS counter - positioned for 720p
        cv2.putText(frame, f"FPS: {self.current_fps:.1f}", (10, h - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)

    def update_fps(self):
        """Update FPS counter."""
        self.fps_counter += 1
        if time.time() - self.fps_start_time >= 1.0:
            self.current_fps = self.fps_counter / \
                (time.time() - self.fps_start_time)
            self.fps_counter = 0
            self.fps_start_time = time.time()

    def start_control(self):
        """Start the advanced webcam control system."""
        args = parse_args()
        cap = cv2.VideoCapture(args.cam_id, apiPreference=cv2.CAP_MSMF)
        if not cap.isOpened():
            logger.error("Could not open webcam")
            return

        # Set camera resolution to 720p
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        logger.info("Advanced webcam control started!")
        logger.info("Press 'y' for pose control, 't' for hand control")

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    logger.error("Failed to capture image from webcam")
                    break

                # Flip frame for mirror effect
                frame = cv2.flip(frame, 1)
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # Process pose and hands
                pose_results = self.pose.process(rgb_frame)
                hand_results = self.hands.process(rgb_frame)

                pose_detected = pose_results.pose_landmarks is not None
                hand_detected = hand_results.multi_hand_landmarks is not None

                # Draw pose landmarks
                if pose_detected:
                    self.mp_drawing.draw_landmarks(
                        frame, pose_results.pose_landmarks,
                        self.mp_pose.POSE_CONNECTIONS,
                        landmark_drawing_spec=self.mp_drawing.DrawingSpec(
                            color=(0, 255, 0), thickness=1),
                        connection_drawing_spec=self.mp_drawing.DrawingSpec(
                            color=(255, 0, 0), thickness=1)
                    )

                # Draw hand landmarks
                if hand_detected:
                    for hand_landmarks in hand_results.multi_hand_landmarks:
                        self.mp_drawing.draw_landmarks(
                            frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
                            landmark_drawing_spec=self.mp_drawing.DrawingSpec(
                                color=(0, 255, 255), thickness=2),
                            connection_drawing_spec=self.mp_drawing.DrawingSpec(
                                color=(255, 255, 0), thickness=2)
                        )

                # Calculate and execute motor movements
                motor_updates = {}
                if pose_detected and (self.pose_control_active or self.hand_control_active):
                    hand_landmarks = None
                    if hand_detected and hand_results.multi_hand_landmarks:
                        hand_landmarks = hand_results.multi_hand_landmarks[0]

                    motor_updates = self.calculate_motor_positions(
                        pose_results.pose_landmarks.landmark, hand_landmarks)

                    # Debug: Print motor calculations
                    if motor_updates:
                        logger.debug(
                            f"Advanced motor calculations: {motor_updates}")

                    # Apply movement threshold and execute
                    filtered_updates = {}
                    for motor_id, position in motor_updates.items():
                        # If this is the first time we see this motor, always move it
                        if motor_id not in self.last_positions:
                            filtered_updates[motor_id] = position
                            self.last_positions[motor_id] = position
                            logger.debug(
                                f"Motor {motor_id}: Initial position -> {position}")
                        else:
                            last_pos = self.last_positions[motor_id]
                            movement_delta = abs(position - last_pos)
                            if movement_delta > self.movement_threshold:
                                filtered_updates[motor_id] = position
                                self.last_positions[motor_id] = position
                                logger.debug(
                                    f"Motor {motor_id}: {last_pos} -> {position} (delta: {movement_delta})")

                    if filtered_updates:
                        try:
                            self.arm.move_multiple_realtime(filtered_updates)
                            logger.debug(
                                f"Advanced control moving motors: {filtered_updates}")
                        except Exception as e:
                            logger.error(f"Motor movement failed: {e}")

                # Draw UI
                self.draw_ui(
                    frame, motor_updates, pose_detected, hand_detected)

                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('h'):
                    logger.info("Returning to home position...")
                    self.arm.return_to_home(speed=300)
                    self.last_positions.clear()
                elif key == ord('s'):
                    self.safety_bounds_active = not self.safety_bounds_active
                    logger.info(
                        f"Safety bounds {'enabled' if self.safety_bounds_active else 'disabled'}")
                elif key == ord('y'):
                    self.pose_control_active = not self.pose_control_active
                    logger.info(
                        f"Pose control {'enabled' if self.pose_control_active else 'disabled'}")
                elif key == ord('t'):
                    self.hand_control_active = not self.hand_control_active
                    logger.info(
                        f"Hand control {'enabled' if self.hand_control_active else 'disabled'}")

                # Update FPS
                self.update_fps()

                cv2.imshow('Robot Arm Control', frame)

        except Exception as e:
            logger.error(f"Error in control loop: {e}")
        finally:
            cap.release()
            cv2.destroyAllWindows()
            self.arm.disconnect()
            logger.info("Advanced webcam control stopped")


def main():
    """Main function with option selection."""
    controller = WebcamControl()
    controller.start_control()


if __name__ == "__main__":
    main()
