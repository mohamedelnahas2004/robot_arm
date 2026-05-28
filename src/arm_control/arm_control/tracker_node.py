import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp
import math
import json

def get_distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

class HandTeleopNode(Node):
    def __init__(self):
        super().__init__('hand_teleop_node')
        self.publisher_ = self.create_publisher(String, '/teleop/commands', 10)
        
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            max_num_hands=1
        )
        
        self.cap = cv2.VideoCapture(0)
        
        # --- TOGGLE MEMORY VARIABLES ---
        self.e_stop_active = False         
        self.is_doing_thumbs_up = False    
        
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        success, frame = self.cap.read()
        if not success:
            return

        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                middle_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
                index_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
                middle_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
                
                hand_depth_metric = get_distance(wrist, middle_mcp) * 100 
                
                gripper_state = "OPEN"
                if index_tip.y > index_mcp.y and middle_tip.y > middle_mcp.y:
                    gripper_state = "CLOSED"

                # 1. Check if the physical gesture is happening right now
                current_gesture_is_thumbs_up = False
                if gripper_state == "CLOSED":
                    palm_length = get_distance(wrist, index_mcp)
                    thumb_extension = get_distance(thumb_tip, index_mcp)
                    if thumb_extension > (palm_length * 0.6) and thumb_tip.y < index_mcp.y:
                        current_gesture_is_thumbs_up = True

                # 2. Toggle the E-STOP state ONLY on the exact frame the gesture starts
                if current_gesture_is_thumbs_up and not self.is_doing_thumbs_up:
                    self.e_stop_active = not self.e_stop_active
                
                # 3. Update the gesture tracker for the next frame
                self.is_doing_thumbs_up = current_gesture_is_thumbs_up

                # 4. ONLY send the ROS message if the E-STOP is OFF
                if not self.e_stop_active:
                    payload = {
                        "x": round(wrist.x, 3),
                        "y": round(wrist.y, 3),
                        "z": round(hand_depth_metric, 3),
                        "gripper": gripper_state,
                        "safety": "GO"
                    }
                    
                    msg = String()
                    msg.data = json.dumps(payload)
                    self.publisher_.publish(msg)
                
                # UI Overlay: Turns Red and shows "MUTED" when E-STOP is active
                safety_text = "E-STOP (MUTED)" if self.e_stop_active else "GO (TRANSMITTING)"
                text_color = (0, 0, 255) if self.e_stop_active else (0, 255, 0)
                cv2.putText(frame, f"Gripper: {gripper_state} | Safety: {safety_text}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)

        cv2.imshow('ROS 2 Teleop Tracker', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HandTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
