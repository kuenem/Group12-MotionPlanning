#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
import moveit_commander

class MoveToJointGoal(Node):
    def __init__(self):
        super().__init__('move_to_joint_goal')
        moveit_commander.roscpp_initialize([])
        self.robot = RobotCommander()
        self.group = MoveGroupCommander("ur_manipulator")  # Default planning group name

        self.move_to_home()

    def move_to_home(self):
        # Define joint goal (in radians)
        joint_goal = [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0]  # example: [-90, -90, -90, -90, 90, 0] degrees
        self.group.go(joint_goal, wait=True)
        self.group.stop()

def main(args=None):

    
    print("Hello from move_to_joint_goal")#new========

    rclpy.init(args=args)
    node = MoveToJointGoal()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
