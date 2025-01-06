#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from trick import RotateNode
import time
from tf2_ros import Buffer, TransformListener

import random, math

class SimpleNavigationNode(Node):
    def __init__(self):
        super().__init__('simple_navigation_node')
        
        # Initialize BasicNavigator (handles goal sending)
        self.navigator = BasicNavigator()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Ensure reliable message delivery
            durability=DurabilityPolicy.VOLATILE,    # Don't store messages for later subscribers
            depth=10                                  # Queue size: control memory usage, higher means more buffer space
        )

        self.voice_command_sub = self.create_subscription(
            String,                     # Message type
            'voice_commands',            # Topic name
            self.listener_callback,      # Callback function
            qos_profile
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # self.turn_node = RotateNode()

    def wait_for_goal(self, x, y, theta):
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

            # self.random_goal()

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
        

    def random_goal(self):
        # Generate random goal coordinates
        goal_x = random.uniform(-4, 4)
        goal_y = random.uniform(-4, 4)
        # goal_theta = random.uniform(-3.14, 3.14)

        # Send the goal
        self.send_goal(goal_x, goal_y, 0.0)

    def listener_callback(self, msg):
        command = msg.data.strip().lower()
        if command == "<listen>":
            self.get_logger().info("Received 'listen' command. Activating listening mode.")
            self.navigator.cancelTask()
            return
        elif command == "<sleep>":
            self.get_logger().info("Received 'sleep' command. Entering sleep mode.")
            # TODO Navigate to bed
            self.random_goal()
            return
        elif command == "<trick>":
            self.get_logger().info("Received 'trick' command. Performing a trick.")
            # TODO Do a trick

            for i in range(random.randint(10, 30)):

                msg = Twist()
                msg.angular.z = 3.0 
                self.cmd_vel_pub.publish(msg)
                time.sleep(0.2)
            return
        else:
            self.get_logger().info(f"Received unknown command: '{msg.data}'. Ignoring.")
            return
        

    def get_current_orientation(self):
        try:
            # Get the transform from 'base_link' to 'odom' (or any other relevant frame)
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())

            # Extract the orientation as a quaternion (x, y, z, w)
            orientation = transform.transform.rotation
            return orientation

        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            return None
        
    def quat_to_yaw(self, quat):
        """Convert quaternion to yaw (rotation around the Z axis)."""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        # Yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)





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

    # navigation_node.random_goal()

    # Spin the node to process callbacks
    rclpy.spin(navigation_node)

    # Shutdown ROS 2 when done
    rclpy.shutdown()

if __name__ == '__main__':
    main()
