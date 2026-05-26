import cv2
import numpy as np
import json
import rclpy
import math
import time  # <--- ADDED TIME MODULE
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            best_effort_qos
        )
        
        self.publisher = self.create_publisher(String, '/vision/targets', 10)
        
        # --- FIXED CALIBRATION CONSTANTS ---
        self.CENTER_X = 0.37   
        self.CENTER_Y = 0.0
        self.TABLE_Z = 0.26
        self.PIXELS_TO_METERS = 0.00124
        
        self.last_published_balls = []
        self.arm_free_time = 0.0
        
        self.get_logger().info("OpenCV Color Masking Active! Waiting for camera feed...")

    def pixel_to_3d(self, px, py, img_width, img_height):
        dx = (px - (img_width / 2.0)) * self.PIXELS_TO_METERS
        dy = ((img_height / 2.0) - py) * self.PIXELS_TO_METERS
        world_x = self.CENTER_X + dx
        world_y = self.CENTER_Y + dy
        return [round(world_x, 3), round(world_y, 3), self.TABLE_Z]

    def find_objects(self, hsv_image, lower_bound, upper_bound):
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        for c in contours:
            if cv2.contourArea(c) > 50:  
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centroids.append((cx, cy))
        return centroids

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, _ = cv_image.shape
            
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            
            lower_red2 = np.array([170, 120, 70])
            upper_red2 = np.array([180, 255, 255])
            
            lower_green = np.array([40, 100, 100])
            upper_green = np.array([80, 255, 255])
            
            lower_blue = np.array([100, 150, 50])
            upper_blue = np.array([140, 255, 255])

            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            green_centroids = self.find_objects(hsv, lower_green, upper_green)
            blue_centroids = self.find_objects(hsv, lower_blue, upper_blue)

            balls_3d = []
            box_3d = None

            for c in contours_red:
                if cv2.contourArea(c) > 50:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), 2)
                        balls_3d.append(self.pixel_to_3d(cx, cy, width, height))

            for (cx, cy) in green_centroids:
                cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), 2)
                balls_3d.append(self.pixel_to_3d(cx, cy, width, height))

            if blue_centroids:
                cx, cy = blue_centroids[0]
                cv2.rectangle(cv_image, (cx-50, cy-50), (cx+50, cy+50), (255, 0, 0), 2)
                box_3d = self.pixel_to_3d(cx, cy, width, height)

            cv2.imshow("Pure OpenCV Targeting", cv_image)
            cv2.waitKey(1)

            # --- THE NEW SPAM FILTER & COOLDOWN LOGIC ---
            if box_3d and len(balls_3d) > 0:
                filtered_balls = []
                for b in balls_3d:
                    distance = math.sqrt((b[0] - box_3d[0])**2 + (b[1] - box_3d[1])**2)
                    if distance > 0.15:  
                        filtered_balls.append(b)
                
                if len(filtered_balls) > 0:
                    current_time = time.time()
                    
                    filtered_balls.sort(key=lambda b: (b[0], b[1]))
                    is_new_scene = False
                    if len(filtered_balls) != len(self.last_published_balls):
                        is_new_scene = True
                    else:
                        for i in range(len(filtered_balls)):
                            dist = math.sqrt((filtered_balls[i][0] - self.last_published_balls[i][0])**2 + 
                                             (filtered_balls[i][1] - self.last_published_balls[i][1])**2)
                            if dist > 0.03: 
                                is_new_scene = True
                                break
                    if is_new_scene and current_time > self.arm_free_time:
                        payload = {"box": box_3d, "balls": filtered_balls}
                        msg = String()
                        msg.data = json.dumps(payload)
                        self.publisher.publish(msg)
                        self.last_published_balls = filtered_balls
                        seconds_needed = len(filtered_balls) * 10.0
                        self.arm_free_time = current_time + seconds_needed
                        
                        self.get_logger().info(f"Published {len(filtered_balls)} targets! Sleeping logic for {seconds_needed}s while arm works...")

        except Exception as e:
            self.get_logger().error(f"Vision failed: {e}")

def main():
    rclpy.init()
    node = VisionProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()