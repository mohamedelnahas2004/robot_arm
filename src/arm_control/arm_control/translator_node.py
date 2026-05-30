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
import numpy as np
import threading
import time

def map_range(value, in_min, in_max, out_min, out_max):
    value = max(min(value, max(in_max, in_min)), min(in_max, in_min))
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def trans(xyz):
    return np.array([
        [1, 0, 0, xyz[0]],
        [0, 1, 0, xyz[1]],
        [0, 0, 1, xyz[2]],
        [0, 0, 0, 1]
    ])
def rot_x(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[1, 0, 0, 0], 
                     [0, c, -s, 0], 
                     [0, s, c, 0], 
                     [0, 0, 0, 1]])
def rot_y(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[c, 0, s, 0], 
                     [0, 1, 0, 0], 
                     [-s, 0, c, 0], 
                     [0, 0, 0, 1]])
def rot_z(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[c, -s, 0, 0], 
                     [s, c, 0, 0], 
                     [0, 0, 1, 0], 
                     [0, 0, 0, 1]])

class IKSolver:
    LIMITS = [
        (-3.14159, 3.14159), # joint_1
        (-1.57, 1.57),       # joint_2
        (-2.09, 0.05),       # joint_3
        (-3.14, 3.14),       # joint_4
        (-1.57, 1.57),       # joint_5
        (-3.14, 3.14),       # joint_6
    ]

    def __init__(self, damping=0.05, max_iter=100, tolerance=0.001, max_step=0.05):
        self.lam = damping
        self.max_iter = max_iter
        self.tol = tolerance
        self.max_step = max_step

    def fk(self, joints):
        q1, q2, q3, q4, q5, q6 = joints
        
        T1 = trans([0.0, 0.000529, 0.111]) @ rot_z(q1)
        T2 = trans([-0.05713, -0.020885, 0.178378]) @ rot_x(q2)
        T3 = trans([0.10013, 0.001311, 0.261007]) @ rot_x(-np.pi/2) @ rot_x(-q3)
        T4 = trans([-0.042999, -0.139695, 0.021705]) @ rot_y(q4)
        T5 = trans([-0.039012, -0.121187, -0.001774]) @ rot_x(-q5)
        T6 = trans([0.039204, -0.033835, 0.0]) @ rot_y(q6)
        T_ee = trans([0.0, -0.006085, 0.0])
        
        T = T1 @ T2 @ T3 @ T4 @ T5 @ T6 @ T_ee
        return T[:3, 3]

    def _jacobian(self, joints):
        eps = 1e-6
        p0 = self.fk(joints)
        J = np.zeros((3, 6))
        for i in range(6):
            j2 = joints.copy()
            j2[i] += eps
            J[:, i] = (self.fk(j2) - p0) / eps
        return J

    def solve(self, target, current_joints):
        joints = np.array(current_joints, dtype=float)
        target = np.array(target)

        for _ in range(self.max_iter):
            ee = self.fk(joints)
            err = target - ee

            if np.linalg.norm(err) < self.tol:
                break

            err_norm = np.linalg.norm(err)
            if err_norm > self.max_step:
                err = err * self.max_step / err_norm

            J = self._jacobian(joints)
            JJT = J @ J.T
            Jinv = J.T @ np.linalg.inv(JJT + self.lam**2 * np.eye(3))
            dq = Jinv @ err

            joints += dq

            for i, (lo, hi) in enumerate(self.LIMITS):
                joints[i] = np.clip(joints[i], lo, hi)

        return joints.tolist()


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

        # --- Macro State Variables ---
        self.current_z = 0.35  # The dynamic Z height
        self.macro_active = False 
        
        self.ik_solver = IKSolver()
        self.current_joints = [0.0, 0.0, -1.0, 0.0, -1.35, 0.0]

        self.smooth_j1 = self.current_joints[0]
        self.smooth_j2 = self.current_joints[1]
        self.smooth_j3 = self.current_joints[2]
        self.smooth_j4 = self.current_joints[3]
        self.smooth_j5 = self.current_joints[4]
        self.smooth_j6 = self.current_joints[5]
        
        self.ALPHA = 0.60

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

    def execute_macro(self, action):
        self.macro_active = True
        
        if action == 'close':
            self.current_z = 0.25 
            time.sleep(0.8) 
            
            self._send_gripper_goal(action)
            while self._gripper_busy:
                time.sleep(0.05) 
                
            self.current_z = 0.35 
            time.sleep(0.5) 
            
        else:
            self._send_gripper_goal(action)
            while self._gripper_busy:
                time.sleep(0.05)
                
        self.macro_active = False

    def command_cb(self, msg):
        try:
            data = json.loads(msg.data)
            cam_x = data['x']  
            cam_y = data['y']  
            safety = data['safety']
            raw_grip = data['gripper']

            if safety == 'E-STOP':
                pass 
            else:
                table_forward = map_range(cam_y, 1.0, 0.0, 0.15, 0.60)
                table_left = map_range(cam_x, 0.0, 1.0, 0.35, -0.35)
                
                target_x = table_left
                target_y = -table_forward
                target_z = self.current_z 
                target_pos = [target_x, target_y, target_z]
                
                solved_joints = self.ik_solver.solve(target_pos, self.current_joints)
                self.current_joints = solved_joints

                self.smooth_j1 = (self.ALPHA * solved_joints[0]) + ((1 - self.ALPHA) * self.smooth_j1)
                self.smooth_j2 = (self.ALPHA * solved_joints[1]) + ((1 - self.ALPHA) * self.smooth_j2)
                self.smooth_j3 = (self.ALPHA * solved_joints[2]) + ((1 - self.ALPHA) * self.smooth_j3)
                self.smooth_j4 = 0.0
                self.smooth_j5 = -1.0  
                self.smooth_j6 = 0.0

            traj = JointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

            point = JointTrajectoryPoint()
            point.positions = [
                self.smooth_j1, 
                self.smooth_j2, 
                self.smooth_j3, 
                self.smooth_j4, 
                self.smooth_j5, 
                self.smooth_j6
            ]
            
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 60_000_000 
            
            traj.points = [point]
            self.traj_pub.publish(traj)

            #MACRO TRIGGER
            if raw_grip != self._last_gripper_state and not self._gripper_busy and not self.macro_active:
                self._last_gripper_state = raw_grip
                target_action = 'open' if raw_grip == 'OPEN' else 'close'
                
                threading.Thread(target=self.execute_macro, args=(target_action,)).start()

        except Exception as e:
            self.get_logger().error(f'IK Translation failed: {e}')

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