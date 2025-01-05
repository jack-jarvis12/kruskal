import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node

class NavigateToCoords(Node):
    def __init__(self):
        super().__init__('navigate_to_coords')

        # Action client for NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, z=0.0, theta=0.0):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # Ensure this is the frame of reference you're using
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # Quaternion orientation (for simplicity, assuming no rotation)
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0

        goal_msg.pose = pose

        # Send the goal to the action server
        self._action_client.wait_for_server()
        self.get_logger().info(f"Sending goal to ({x}, {y}, {theta})")
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    # Coordinates to move to (you can adjust this as needed)
    coordinates = [(1.0, 2.0), (2.0, 3.0), (3.0, 4.0)]

    navigator = NavigateToCoords()

    for coord in coordinates:
        x, y = coord
        navigator.send_goal(x, y)
        # Wait some time before sending the next goal
        rclpy.spin_once(navigator)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
