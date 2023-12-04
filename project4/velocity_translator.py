import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class VelocityTranslator(Node):
    def __init__(self):
        super().__init__('velocity_translator')

        self.vl_publisher = self.create_publisher(Float64, '/vl', 10)
        self.vr_publisher = self.create_publisher(Float64, '/vr', 10)
        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate wheel velocities based on linear and angular components
        vl = linear_x - angular_z / 2.0
        vr = linear_x + angular_z / 2.0

         # Publish wheel velocities
        vl_msg = Float64()
        vl_msg.data = vl
        vr_msg = Float64()
        vr_msg.data = vr

        self.vl_publisher.publish(vl_msg)
        self.vr_publisher.publish(vr_msg)

def main():
    rclpy.init()
    velocity_translator = VelocityTranslator()
    rclpy.spin(velocity_translator)
    velocity_translator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()