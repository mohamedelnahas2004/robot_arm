import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints,
                              PositionConstraint, OrientationConstraint,
                              BoundingVolume, MoveItErrorCodes)
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import math


def get_downward_quaternion(x, y):
    roll  = math.pi / 2.0
    pitch = 0.0
    yaw   = math.atan2(y, x) + (math.pi / 2.0)
    cy = math.cos(yaw * 0.5);   sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return qx, qy, qz, qw


class TeleopInitNode(Node):
    def __init__(self):
        super().__init__('teleop_init_node')

        self.ready_x = 0.0
        self.ready_y = -0.4
        self.ready_z = 0.45

        self._client = ActionClient(self, MoveGroup, '/move_action')
        self._goal_sent = False

        # Poll every 0.5s until move_group is available, then send goal once
        self._startup_timer = self.create_timer(0.5, self._wait_and_send)
        self.get_logger().info('Teleop Init: Waiting for move_group action server...')

    def _wait_and_send(self):
        if self._goal_sent:
            return
        if not self._client.server_is_ready():
            return  
        
        self._startup_timer.cancel()
        self._goal_sent = True
        self.get_logger().info('Teleop Init: Moving arm to ready pose...')
        self._send_goal()

    def _send_goal(self):
        qx, qy, qz, qw = get_downward_quaternion(self.ready_x, self.ready_y)

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = self.ready_x
        target_pose.pose.position.y = self.ready_y
        target_pose.pose.position.z = self.ready_z
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'Link_6_1'
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]
        bv.primitives.append(sp)
        bv.primitive_poses.append(target_pose.pose)
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'base_link'
        ori_constraint.link_name = 'Link_6_1'
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pos_constraint)
        goal_constraints.orientation_constraints.append(ori_constraint)

        request = MotionPlanRequest()
        request.group_name = 'arm'
        request.goal_constraints.append(goal_constraints)
        request.num_planning_attempts = 5
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5
        request.planner_id = 'RRTConnectkConfigDefault'

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self._send_goal_future = self._client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Teleop Init: Goal rejected!')
            return
        self.get_logger().info('Teleop Init: Goal accepted, executing...')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(
                'Ready pose reached! Show your hand to start teleop.')
        else:
            self.get_logger().error(
                f'Failed to reach ready pose. Error code: {result.error_code.val}')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopInitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()