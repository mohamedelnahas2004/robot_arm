import math
import json
import time
import rclpy
from moveit.planning import MoveItPy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class PickAndPlace:
    
    def __init__(self):
        self.moveit_robot = MoveItPy(node_name="dynamic_pnp")
        self.arm = self.moveit_robot.get_planning_component("arm")
        self.gripper = self.moveit_robot.get_planning_component("gripper")
        self.is_busy = False
        
        self.x_offset = 0.02 
        self.y_offset = 0.0

        self.node = rclpy.create_node("vision_subscriber")
        self.subscription = self.node.create_subscription(
            String,
            '/vision/targets',
            self.vision_callback,
            10
        )
        self.node.get_logger().info("System ready. Waiting for targets...")

    def move_gripper(self, state_name):
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name=state_name)
        plan = self.gripper.plan()
        if plan and plan.trajectory:
            self.moveit_robot.execute(plan.trajectory, controllers=[])
            return True
        return False

    def get_downward_quaternion(self, x, y):
        roll = math.pi / 2.0
        pitch = 0.0
        yaw = math.atan2(y, x) + (math.pi / 2.0)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw

    def move_arm_to_pose(self, x, y, z):
        target = PoseStamped()
        target.header.frame_id = "world" 
        
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        
        qx, qy, qz, qw = self.get_downward_quaternion(x, y)
        target.pose.orientation.x = qx
        target.pose.orientation.y = qy
        target.pose.orientation.z = qz
        target.pose.orientation.w = qw

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=target, pose_link="Link_6_1")
        
        plan = self.arm.plan()
        if plan and plan.trajectory:
            self.moveit_robot.execute(plan.trajectory, controllers=[])
            return True
        
        self.node.get_logger().error(f"IK Failed! Cannot reach X:{x} Y:{y} Z:{z}")
        return False
    
    def move_to_home(self):
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="home") 
        plan = self.arm.plan()
        if plan and plan.trajectory:
            self.moveit_robot.execute(plan.trajectory, controllers=[])
            
    def vision_callback(self, msg):
        if self.is_busy: return
        self.is_busy = True

        try:
            targets = json.loads(msg.data)
            box = targets['box']
            balls = targets['balls']

            for i, ball in enumerate(balls):
                bx, by, bz = ball[0], ball[1], ball[2]
                box_x, box_y, box_z = box[0], box[1], box[2]
                
                bx = bx + self.x_offset
                by = by + self.y_offset
            
                if not self.move_arm_to_pose(bx, by, bz + 0.10): continue
                time.sleep(0.5)
                
                self.move_gripper("open")
                time.sleep(0.5)
                
                if not self.move_arm_to_pose(bx, by, bz): continue
                time.sleep(0.5)
                
                self.move_gripper("close")
                time.sleep(0.5)
                
                if not self.move_arm_to_pose(bx, by, bz + 0.15): continue
                time.sleep(0.5)
                
                if not self.move_arm_to_pose(box_x, box_y, box_z + 0.15): continue
                time.sleep(0.5)
                
                self.move_gripper("open")
                time.sleep(0.5)

            self.node.get_logger().info("Job complete! Returning home...")
            self.move_to_home()
            self.node.get_logger().info("Sequence Complete")

        except Exception as e:
            self.node.get_logger().error(f"Error: {e}")
        finally:
            self.is_busy = False

def main():
    rclpy.init()
    pnp_system = PickAndPlace()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(pnp_system.node) 
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()