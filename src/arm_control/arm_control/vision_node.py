import cv2
import numpy as np
import json
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
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
        self.subscription = self.create_subscription(Image,'/camera/image_raw',self.image_callback,best_effort_qos)
        self.srv = self.create_service(Trigger, '/vision/get_targets', self.target_service_callback)
        self.CENTER_X = 0.37
        self.CENTER_Y = 0.0
        self.TABLE_Z = 0.27
        self.PIXELS_TO_METERS = 0.00124
        
        self.latest_world_data = None
        self.latest_total_balls = 0
        
        self.get_logger().info("OpenCV Vision Server Active! Waiting for arm requests...")

    def target_service_callback(self, request, response):
        if self.latest_world_data is not None and self.latest_total_balls > 0:
            response.success = True
            response.message = json.dumps(self.latest_world_data)
            self.get_logger().info("Arm requested targets. Coordinates sent!")
        else:
            response.success = False
            response.message = "{}"
            self.get_logger().info("Arm requested targets, but no valid balls were found.")
        return response

    def pixel_to_3d(self, px, py, img_width, img_height):
        dx = (px - (img_width / 2.0)) * self.PIXELS_TO_METERS
        dy = ((img_height / 2.0) - py) * self.PIXELS_TO_METERS
        world_x = self.CENTER_X + dx
        world_y = self.CENTER_Y + dy
        return [round(world_x, 3), round(world_y, 3), self.TABLE_Z]

    def find_objects(self, hsv_image, lower_bound, upper_bound):
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = cnts[0] if len(cnts) == 2 else cnts[1]
        
        balls_px = []
        boxes_px = []
        
        for c in contours:
            area = cv2.contourArea(c)
            if area > 50:  
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    if area > 3000:  
                        boxes_px.append((cx, cy))
                    elif area < 1500:
                        balls_px.append((cx, cy))
                        
        return balls_px, boxes_px

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            height = cv_image.shape[0]
            width = cv_image.shape[1]
            
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
            
            cnts_red = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours_red = cnts_red[0] if len(cnts_red) == 2 else cnts_red[1]
            
            red_balls_px = []
            red_boxes_px = []
            
            for c in contours_red:
                area = cv2.contourArea(c)
                if area > 50:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        if area > 3000: 
                            red_boxes_px.append((cx, cy))
                        elif area < 1500: 
                            red_balls_px.append((cx, cy))

            green_balls_px, green_boxes_px = self.find_objects(hsv, lower_green, upper_green)
            blue_balls_px, blue_boxes_px = self.find_objects(hsv, lower_blue, upper_blue)

            def process_items(px_list, is_box, bgr_color):
                items_3d = []
                if not px_list: 
                    return items_3d
                for coords in px_list:
                    c_x = coords[0]
                    c_y = coords[1]
                    if is_box:
                        cv2.rectangle(cv_image, (c_x-30, c_y-30), (c_x+30, c_y+30), bgr_color, 2)
                    else:
                        cv2.circle(cv_image, (c_x, c_y), 10, bgr_color, 2)
                    items_3d.append(self.pixel_to_3d(c_x, c_y, width, height))
                return items_3d

            red_balls_3d = process_items(red_balls_px, False, (0, 0, 255))
            red_boxes_3d = process_items(red_boxes_px, True, (0, 0, 255))
            
            green_balls_3d = process_items(green_balls_px, False, (0, 255, 0))
            green_boxes_3d = process_items(green_boxes_px, True, (0, 255, 0))
            
            blue_balls_3d = process_items(blue_balls_px, False, (255, 0, 0))
            blue_boxes_3d = process_items(blue_boxes_px, True, (255, 0, 0))

            cv2.imshow("Pure OpenCV Targeting", cv_image)
            cv2.waitKey(1)

            all_boxes_3d = red_boxes_3d + green_boxes_3d + blue_boxes_3d
            IGNORE_RADIUS = 0.10

            def in_exclusion_zone(ball_3d):
                for box_3d in all_boxes_3d:
                    dist = math.sqrt((ball_3d[0] - box_3d[0])**2 + (ball_3d[1] - box_3d[1])**2)
                    if dist < IGNORE_RADIUS:
                        return True
                return False

            final_red_balls = [b for b in red_balls_3d if not in_exclusion_zone(b)]
            final_green_balls = [b for b in green_balls_3d if not in_exclusion_zone(b)]
            final_blue_balls = [b for b in blue_balls_3d if not in_exclusion_zone(b)]

            self.latest_world_data = {
                "red": {"box": red_boxes_3d[0] if len(red_boxes_3d) > 0 else None, "balls": final_red_balls},
                "green": {"box": green_boxes_3d[0] if len(green_boxes_3d) > 0 else None, "balls": final_green_balls},
                "blue": {"box": blue_boxes_3d[0] if len(blue_boxes_3d) > 0 else None, "balls": final_blue_balls}
            }
            
            self.latest_total_balls = len(final_red_balls) + len(final_green_balls) + len(final_blue_balls)

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