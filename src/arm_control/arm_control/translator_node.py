import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import json

def map_range(value, in_min, in_max, out_min, out_max):
    value = max(min(value, max(in_max, in_min)), min(in_max, in_min))
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class CustomArmDecoupledNode(Node):
    def __init__(self):
        super().__init__('custom_arm_decoupled_node')
        self.cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            String, '/teleop/commands', self.command_cb, 1, callback_group=self.cb_group)

        self.traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self._gripper_client = ActionClient(self, MoveGroup, '/move_action', callback_group=self.cb_group)
        self._last_gripper_state = "OPEN"
        self._gripper_busy = False

        self.smooth_j1 = 0.0
        self.smooth_j2 = 0.0
        self.smooth_j3 = -1.0
        self.ALPHA = 0.20

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
        result_future.add_done_callback(lambda f: setattr(self, '_gripper_busy', False))

    def command_cb(self, msg):
        try:
            data = json.loads(msg.data)
            cam_x = data['x']  
            cam_y = data['y']  
            safety = data['safety']
            raw_grip = data['gripper']

            if safety == 'E-STOP':
                target_j1 = self.smooth_j1
                target_j2 = self.smooth_j2
                target_j3 = self.smooth_j3
            else:
                # 1. PAN (Left/Right)
                target_j1 = map_range(cam_x, 0.0, 1.0, 1.5, -1.5)

                # ── 2. REACH (Up/Down) FIXED RANGE ──
                # Tuned the extremes down so it behaves like a comfortable lean over the table
                target_j2 = map_range(cam_y, 0.0, 1.0, 1.5, 0.55)
                target_j3 = map_range(cam_y, 0.3, 0.6, -1.3, -0.2)

            self.smooth_j1 = (self.ALPHA * target_j1) + ((1 - self.ALPHA) * self.smooth_j1)
            self.smooth_j2 = (self.ALPHA * target_j2) + ((1 - self.ALPHA) * self.smooth_j2)
            self.smooth_j3 = (self.ALPHA * target_j3) + ((1 - self.ALPHA) * self.smooth_j3)

            traj = JointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

            WRIST_PITCH = -1.57  

            point = JointTrajectoryPoint()
            point.positions = [self.smooth_j1, self.smooth_j2, self.smooth_j3, 0.0, WRIST_PITCH, 0.0]
            
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 60_000_000 
            
            traj.points = [point]
            self.traj_pub.publish(traj)

            if raw_grip != self._last_gripper_state and not self._gripper_busy:
                target = 'open' if raw_grip == 'OPEN' else 'close'
                self._send_gripper_goal(target)
                self._last_gripper_state = raw_grip

        except Exception as e:
            self.get_logger().error(f'Decoupled Translation failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CustomArmDecoupledNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()