import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Parameters for the controller
        self.target_distance_from_wall = 0.5  # Adjust as needed
        self.linear_speed = 0.35  # Adjust as needed
        self.turn_around_threshold = 0.4  # Adjust as needed

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
        angular_z = -0.2 * error

        frontmost_range_avg = 0

        for i in range(len(ranges)):
            if (i > center_index - 5) and (i < center_index + 5):
                frontmost_range_avg += ranges[i]

        frontmost_range_avg = frontmost_range_avg / 10

        # Check the frontmost laser scan range
        frontmost_range = ranges[center_index]

        # If the frontmost range is below the threshold, stop linear motion and turn around
        if frontmost_range < self.turn_around_threshold:
            print("Goal Reached!")
            linear_x = 0.0
            angular_z = 1.0  # High angular speed for turning around
        else:
            linear_x = self.linear_speed

        # Create Twist message with linear velocity and angular velocity
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_x
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