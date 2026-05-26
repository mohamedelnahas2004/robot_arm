import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int8
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints,
                              JointConstraint, MoveItErrorCodes)
from rclpy.action import ActionClient
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
import json
import math
import time

# --- HELPER FUNCTIONS ---
def deadband(value, threshold):
    if abs(value) < threshold:
        return 0.0
    return value

def map_range(value, in_min, in_max, out_min, out_max):
    value = max(min(value, in_max), in_min)
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class ServoTeleopTranslatorNode(Node):
    def __init__(self):
        super().__init__('servo_teleop_translator_node')

        self.subscription = self.create_subscription(
            String, '/teleop/commands', self.command_callback, 10)

        self.twist_publisher = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.gripper_publisher = self.create_publisher(
            Bool, '/teleop/gripper_cmd', 10)

        self.status_publisher = self.create_publisher(
            Int8, '/servo_node/pause_servo', 10)

        self.servo_command_type_client = self.create_client(
            ServoCommandType, '/servo_node/switch_command_type')
        self.pause_client = self.create_client(
            SetBool, '/servo_node/pause_servo')
        self._servo_enabled = False
        self.create_timer(2.0, self._enable_servo_once)

        self._gripper_client = ActionClient(self, MoveGroup, '/move_action')
        self._last_gripper_state = "OPEN"   
        self._gripper_busy = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.dive_state = "HOVER"
        self.HOVER_Z = 0.40  
        self.DIVE_Z = 0.22   
        self.grab_timer = 0.0

        self.is_calibrated = False
        self.last_msg_time = time.time()
        self.center_cam_x = 0.0
        self.center_cam_y = 0.0

        self.SENSE_XY = 0.25   
        self.MAX_LINEAR_VEL = 0.8  

        self.last_hand_time = time.time()
        self.create_timer(0.01, self.safety_timer_callback)

        self.get_logger().info('Absolute Controller Active. Show hand to set Zero Point.')

    def _enable_servo_once(self):
        if self._servo_enabled: return
        if not self.servo_command_type_client.wait_for_service(timeout_sec=1.0):
            return
        req = ServoCommandType.Request()
        req.command_type = 1  # TWIST
        future = self.servo_command_type_client.call_async(req)
        future.add_done_callback(self._servo_enable_response)
        self._servo_enabled = True

    def _servo_enable_response(self, future):
        try:
            if future.result().success:
                self.get_logger().info('Servo TWIST mode enabled! Unpausing...')
                self._unpause_servo()
            else:
                self._servo_enabled = False
        except Exception:
            self._servo_enabled = False

    def _unpause_servo(self):
        if not self.pause_client.wait_for_service(timeout_sec=1.0): return
        req = SetBool.Request()
        req.data = False  
        self.pause_client.call_async(req)

    def safety_timer_callback(self):
        if time.time() - self.last_hand_time > 0.1:
            self._publish_zero_twist()

    def _publish_zero_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.twist_publisher.publish(msg)

    def _send_gripper_goal(self, state_name):
        if self._gripper_busy or not self._gripper_client.server_is_ready():
            return

        self._gripper_busy = True
        jaw_val = -0.002 if state_name == 'open' else 0.015

        constraints = Constraints()
        for joint in ['jaw1', 'jaw2']:
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = jaw_val
            jc.tolerance_above = 0.005
            jc.tolerance_below = 0.005
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request = MotionPlanRequest()
        request.group_name = 'gripper'
        request.goal_constraints.append(constraints)
        request.num_planning_attempts = 3
        request.allowed_planning_time = 3.0

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False

        future = self._gripper_client.send_goal_async(goal)
        future.add_done_callback(self._gripper_goal_response)
        self.get_logger().info(f'Gripper → {state_name}')

    def _gripper_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._gripper_busy = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_result)

    def _gripper_result(self, future):
        self._gripper_busy = False

    def command_callback(self, msg):
        try:
            current_time = time.time()
            self.last_hand_time = current_time

            if current_time - self.last_msg_time > 2.0:
                self.is_calibrated = False
                self.get_logger().warn('Hand lost! Waiting for new Zero Point...')

            self.last_msg_time = current_time

            data          = json.loads(msg.data)
            cam_x         = data['x']
            cam_y         = data['y']
            gripper_state = data['gripper']
            safety        = data['safety']

            if safety == 'E-STOP':
                self._publish_zero_twist()
                return

            if not self.is_calibrated:
                self.center_cam_x = cam_x
                self.center_cam_y = cam_y
                self.is_calibrated = True
                self.get_logger().info('ZERO POINT LOCKED. Driving arm.')
                return

            # ── 1. Absolute Position Mapping ──────────────────────────────────
            c_x_min, c_x_max = self.center_cam_x - self.SENSE_XY, self.center_cam_x + self.SENSE_XY
            c_y_min, c_y_max = self.center_cam_y - self.SENSE_XY, self.center_cam_y + self.SENSE_XY

            target_x = map_range(cam_y, c_y_min, c_y_max, 0.2, 0.6)
            target_y = map_range(cam_x, c_x_min, c_x_max, 0.4, -0.4)

            try:
                trans = self.tf_buffer.lookup_transform('base_link', 'Link_6_1', rclpy.time.Time())
                curr_x = trans.transform.translation.x
                curr_y = trans.transform.translation.y
                curr_z = trans.transform.translation.z
            except Exception:
                curr_x, curr_y, curr_z = target_x, target_y, self.HOVER_Z

            target_z = self.HOVER_Z

            if gripper_state == "OPEN":
                self.dive_state = "HOVER"
                target_z = self.HOVER_Z
                if self._last_gripper_state != "OPEN":
                    self._send_gripper_goal("open")
                    self._last_gripper_state = "OPEN"

            elif gripper_state == "CLOSED":
                if self.dive_state == "HOVER":
                    self.dive_state = "DIVING"

                if self.dive_state == "DIVING":
                    target_z = self.DIVE_Z
                    if curr_z <= (self.DIVE_Z + 0.015): 
                        self.dive_state = "GRABBING"
                        self._send_gripper_goal("close")
                        self._last_gripper_state = "CLOSED"
                        self.grab_timer = time.time()

                if self.dive_state == "GRABBING":
                    target_z = self.DIVE_Z
                    if time.time() - self.grab_timer > 0.8: 
                        self.dive_state = "HOLDING"

                if self.dive_state == "HOLDING":
                    target_z = self.HOVER_Z

            Kp_xy = 6.0  
            Kp_z  = 4.0  

            vx = (target_x - curr_x) * Kp_xy
            vy = (target_y - curr_y) * Kp_xy
            vz = (target_z - curr_z) * Kp_z

            vx = max(-self.MAX_LINEAR_VEL, min(self.MAX_LINEAR_VEL, vx))
            vy = max(-self.MAX_LINEAR_VEL, min(self.MAX_LINEAR_VEL, vy))
            vz = max(-self.MAX_LINEAR_VEL, min(self.MAX_LINEAR_VEL, vz))

            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.linear.x = float(vx)
            twist_msg.twist.linear.y = float(vy)
            twist_msg.twist.linear.z = float(vz)
            
            self.twist_publisher.publish(twist_msg)

        except Exception as e:
            self.get_logger().error(f'Translation failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoTeleopTranslatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()