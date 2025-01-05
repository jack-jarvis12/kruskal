#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

import random

class SimpleNavigationNode(Node):
    def __init__(self):
        super().__init__('simple_navigation_node')
        
        # Initialize BasicNavigator (handles goal sending)
        self.navigator = BasicNavigator()

    def send_goal(self, x, y, theta):
        # Create a PoseStamped message (goal pose)
        goal = PoseStamped()
        goal.header.frame_id = 'map'  # or 'odom' depending on your system
        goal.header.stamp = self.get_clock().now().to_msg()
        
        # Set the goal position
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = theta

        # Send the goal to the navigator
        self.navigator.goToPose(goal)

        # Wait until the robot reaches the goal or fails
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback.navigation_time.sec > 10:
                self.navigator.cancelTask()
        
        if self.navigator.isTaskComplete():

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Goal reached!")
            elif result == TaskResult.CANCELED:
                self.get_logger().info('Goal was canceled!')
            elif result == TaskResult.FAILED:
                self.get_logger().info('Goal failed!')

            self.random_goal()

    def random_goal(self):
        # Generate random goal coordinates
        goal_x = random.uniform(-4, 4)
        goal_y = random.uniform(-4, 4)
        # goal_theta = random.uniform(-3.14, 3.14)

        # Send the goal
        self.send_goal(goal_x, goal_y, 0.0)

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of your node
    navigation_node = SimpleNavigationNode()

    # Define your goal (x, y, theta in radians)
    # goal_x = -5.0  # meters
    # goal_y = -5.0  # meters
    # goal_theta = 0.0  # radians (0 for facing straight ahead)

    # # Send goal
    # navigation_node.send_goal(goal_x, goal_y, goal_theta)

    navigation_node.random_goal()

    # Spin the node to process callbacks
    rclpy.spin(navigation_node)

    # Shutdown ROS 2 when done
    rclpy.shutdown()

if __name__ == '__main__':
    main()
