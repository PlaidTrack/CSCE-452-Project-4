import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Parameters for the controller
        self.target_distance_from_wall = 0.5  # Adjust as needed
        self.linear_speed = 0.2  # Adjust as needed

    def scan_callback(self, msg):
        # Process laser scan data and generate velocity commands
        cmd_vel_msg = self.calculate_velocity_commands(msg)
        self.cmd_vel_publisher.publish(cmd_vel_msg)

    def calculate_velocity_commands(self, scan_msg):
        # Extract laser ranges from the scan message
        ranges = np.array(scan_msg.ranges)

        # Calculate the center index of the laser scan data
        center_index = len(ranges) // 2

        # Calculate the average range on each side of the robot
        left_avg_range = np.mean(ranges[:center_index])
        right_avg_range = np.mean(ranges[center_index:])

        # Calculate the error between left and right average ranges
        error = left_avg_range - right_avg_range

        # Proportional control to maintain the largest range in the center
        angular_z = -0.1 * error

        # Create Twist message with linear velocity and angular velocity
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.linear_speed
        cmd_vel_msg.angular.z = angular_z

        return cmd_vel_msg

def main():
    rclpy.init()
    navigation_controller = NavigationController()
    rclpy.spin(navigation_controller)
    navigation_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()