# The load_disc_robot method reads a file that describes a disc-shaped robot
# and returns a dictionary describing the properties of that robot.

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from tf2_ros import TransformBroadcaster
from math import cos, sin
import time

class SimulateRobot(Node):
    def __init__(self):
        super().__init__('simulate_robot')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('radius', 0.2),
                ('height', 0.06),
                ('distance', 0.3),
                ('velocity_error_variance', 0.01),
            ]
        )

        input_robot_yaml = '/root/ros2_project4/src/project4/robot_data/normal_robot.yaml'

        self.robot_radius = 0.0
        self.robot_height = 0.0
        self.robot_wheel_distance = 0.0
        self.robot_error_variance_l = 0.0
        self.robot_error_variance_r = 0.0
        self.robot = self.load_disc_robot(input_robot_yaml)

        self.get_logger().info("Differential Drive Simulator Initialized")

        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.last_velocity_time = time.time()
        self.last_received_vl = 0.0
        self.last_received_vr = 0.0

        self.vl_sub = self.create_subscription(
            Float64, '/vl', self.vl_callback, 10)
        self.vr_sub = self.create_subscription(
            Float64, '/vr', self.vr_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.update_pose)

    def vl_callback(self, msg):
        self.last_velocity_time = time.time()
        self.last_received_vl = msg.data

    def vr_callback(self, msg):
        self.last_velocity_time = time.time()
        self.last_received_vr = msg.data

    def update_pose(self):
        if time.time() - self.last_velocity_time > 1.0:
            # Stop moving if no velocity commands are received for one second
            vl = vr = 0.0
        else:
            vl = self.last_received_vl
            vr = self.last_received_vr

        self.update_pose_from_velocities(vl, vr)

    def update_pose_from_velocities(self, vl, vr):
        # Apply random multiplicative error to velocities
        vl *= (1.0 + self.robot_error_variance_l)
        vr *= (1.0 + self.robot_error_variance_r)

        # Implement differential drive kinematics
        radius = self.robot_radius
        distance = self.robot_wheel_distance

        dt = 0.1  # Time step for integration

        omega = (vr - vl) / distance
        v = (vr + vl) / 2.0

        dx = v * cos(self.current_pose['theta']) * dt
        dy = v * sin(self.current_pose['theta']) * dt
        dtheta = omega * dt

        # Update current pose
        self.current_pose['x'] += dx
        self.current_pose['y'] += dy
        self.current_pose['theta'] += dtheta

        # Broadcast transform from world to base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation = Vector3(x=self.current_pose['x'], y=self.current_pose['y'], z=0.0)
        q = self.get_quaternion(self.current_pose['theta'])
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.tf_broadcaster.sendTransform(t)

    def get_quaternion(self, angle):
        return [0.0, 0.0, sin(angle / 2.0), cos(angle / 2.0)]
    
    def load_disc_robot(self, file_name):
        with open(file_name, 'r') as f:
            robot = yaml.safe_load(f)
        robot['urdf'] = self.disc_robot_urdf(robot)
        print(self.robot_radius)
        print(self.robot_height)
        return robot

    def disc_robot_urdf(self, robot):
        self.robot_radius = robot['body']['radius']
        self.robot_height = robot['body']['height']
        self.robot_wheel_distance = robot['wheels']['distance']
        self.robot_error_variance_l = robot['wheels']['error_variance_left']
        self.robot_error_variance_r = robot['wheels']['error_variance_right']


# write a new urdf file based on robot data
def write_new_urdf(yaml_file, output_urdf):
    with open(yaml_file, 'r') as file:
        robot_description = yaml.safe_load(file)
    
    #f = open("/root/ros2_project1/src/project4/urdf/new_robot.urdf", "w")
    f = open(output_urdf, "w")

    f.write("<?xml version=\"1.0\"?>\n")
    f.write("<robot name=\"materials\">\n\n")

    f.write("  <material name=\"blue\">\n")
    f.write("    <color rgba=\"0.5 0.5 1 1\"/>\n")
    f.write("  </material>\n\n")

    f.write("<link name=\"base_link\">\n")
    f.write("    <visual>\n")
    f.write("      <geometry>\n")
    f.write(f"        <cylinder length=\"{robot_description['body']['height']}\" radius=\"{robot_description['body']['radius']}\"/>\n")
    f.write("      </geometry>\n")
    f.write("      <material name=\"blue\"/>\n")
    f.write("    </visual>\n")
    f.write("  </link>\n\n")

    f.write("</robot>")



def main():
    rclpy.init()
    simulator = SimulateRobot()
    
    input_robot_yaml = '/root/ros2_project4/src/project4/robot_data/normal_robot.yaml'
    output_robot_urdf = '/root/ros2_project4/src/project4/urdf/new_robot.urdf.xml'
    write_new_urdf(input_robot_yaml, output_robot_urdf)

    rclpy.spin(simulator)
    simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()