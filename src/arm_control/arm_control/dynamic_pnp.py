import math
import json
import time
import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

class PickAndPlace:
    
    def __init__(self):
        self.moveit_robot = MoveItPy(node_name="dynamic_pnp")
        self.arm = self.moveit_robot.get_planning_component("arm")
        self.gripper = self.moveit_robot.get_planning_component("gripper")
        
        self.node = rclpy.create_node("vision_client")
        self.cli = self.node.create_client(Trigger, '/vision/get_targets')
        self.is_busy = False
        self.x_offset = 0.02

        # A timer that acts as our heartbeat. Every 3 seconds, we check if we should ask for work.
        self.timer = self.node.create_timer(3.0, self.request_targets)
        self.node.get_logger().info("System ready. Waiting for sorting targets...")

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
            
    def request_targets(self):
        """ The heartbeat function. Asks the camera for data if the arm is idle. """
        if self.is_busy:
            return

        if not self.cli.wait_for_service(timeout_sec=1.0):
            return # meh not now

        # خلص خش على اللي بعده
        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.process_vision_response)

    def process_vision_response(self, future):
        """ The execution function. Runs when the camera replies. """
        if self.is_busy:
            return

        try:
            response = future.result()
            
            if not response.success:
                return 
            self.is_busy = True
            world_data = json.loads(response.message)
            tasks = []
            for color in ["red", "green", "blue"]:
                box = world_data[color].get("box")
                balls = world_data[color].get("balls", [])
                if box:
                    for b in balls:
                        tasks.append({"pick": b, "drop": box})

            if len(tasks) == 0:
                self.is_busy = False
                return

            self.node.get_logger().info(f"Target Acquired! Executing {len(tasks)} tasks...")
            time.sleep(1.0)
            
            # EXECUTE TASKS
            for task in tasks:
                bx, by, bz = task["pick"][0], task["pick"][1], task["pick"][2]
                box_x, box_y, box_z = task["drop"][0], task["drop"][1], task["drop"][2]
                
                bx = bx + self.x_offset
            
                if not self.move_arm_to_pose(bx, by, bz + 0.15): continue
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

            self.node.get_logger().info("Sorting complete! Returning home...")
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