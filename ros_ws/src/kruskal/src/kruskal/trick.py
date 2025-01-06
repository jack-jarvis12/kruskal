import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RotateNode(Node):
    def __init__(self):
        super().__init__('rotate_node')

        # Create a publisher for the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create a timer (but we'll control the publishing manually later)
        self.timer = None
        
        # Create a Twist message for rotation
        self.msg = Twist()
        self.msg.angular.z = 1.0  # Rotation speed: 1 rad/s

        # Variable to control the publishing state
        self.is_publishing = False
        
        self.get_logger().info("Node initialized. Use 'start_publishing()' and 'stop_publishing()' to control publishing.")

    def start_publishing(self):
        if not self.is_publishing:
            self.get_logger().info('Starting to publish rotation command at 5 Hz.')
            self.timer = self.create_timer(0.2, self.timer_callback)  # 5 Hz
            self.is_publishing = True

    def stop_publishing(self):
        if self.is_publishing:
            self.get_logger().info('Stopping publishing.')
            self.timer.cancel()  # Stop the timer from triggering the callback
            self.is_publishing = False
            # Optionally, stop publishing immediately by sending zero velocities
            self.publish_zero_velocity()

    def timer_callback(self):
        # Publish the rotation message
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing: "%s"', self.msg)

    def publish_zero_velocity(self):
        # Send a zero velocity message to stop any motion
        zero_msg = Twist()
        self.publisher_.publish(zero_msg)
        self.get_logger().info('Publishing: Stop command')