# The load_disc_robot method reads a file that describes a disc-shaped robot
# and returns a dictionary describing the properties of that robot.

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Header
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from math import cos, sin
import numpy as np
import time

class SimulateRobot(Node):
    def __init__(self):
        super().__init__('simulate_robot')
        
        input_robot_yaml = '/root/ros2_project4/src/project4/robot_data/normal_robot.yaml'
        input_world = '/root/ros2_project4/src/project4/world_data/brick.world'

        # Robot body
        self.robot_radius = 0.0
        self.robot_height = 0.0

        # Robot wheels
        self.robot_wheel_distance = 0.0
        self.robot_error_variance_l = 0.0
        self.robot_error_variance_r = 0.0

        # Robot laser scanner
        self.laser_rate = 0
        self.laser_count = 0
        self.laser_angle_min = 0.0
        self.laser_angle_max = 0.0
        self.laser_range_min = 0.0
        self.laser_range_max = 0.0
        self.laser_error_variance = 0.0
        self.laser_fail_probaability = 0.0

        self.robot = self.load_disc_robot(input_robot_yaml)

        self.get_logger().info("Differential Drive Simulator Initialized")
        
        # Each cell is a square with this side length,
        # measured in meters.
        self.world_resolution = 0.0
        self.map_width = 0
        self.map_height = 0
        
        # The robot starts here, given as [x, y, theta], with
        # x and y in meters and theta in radians.
        self.robot_initial_pose = [0.0, 0.0, 0.0]   # Fetched from world file
        
        # Where the obstacles be
        self.world_map = ""

        self.load_world(input_world)

        self.current_pose = {'x': self.world_initial_pose[0], 'y': self.world_initial_pose[1], 'theta': self.world_initial_pose[2]}
        self.last_velocity_time = time.time()
        self.last_received_vl = 0.0
        self.last_received_vr = 0.0

        self.vl_sub = self.create_subscription(
            Float64, '/vl', self.vl_callback, 10)
        self.vr_sub = self.create_subscription(
            Float64, '/vr', self.vr_callback, 10)
        
        self.map_publisher = self.create_publisher(
            OccupancyGrid, '/map', 10)

        self.scan_publisher = self.create_publisher(
            LaserScan, '/scan', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Parse the map string and convert it to an occupancy grid
        self.occupancy_grid = self.parse_map_string()

        self.publish_map()
        self.timer = self.create_timer(0.1, self.update_pose)
        #self.timer = self.create_timer(1.0, self.publish_map)

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

        # Collision check
        if not self.check_collision(vl, vr):
            self.update_pose_from_velocities(vl, vr)
        
        self.publish_laser_scan()
    
    def check_collision(self, vl, vr):
        # Simulate the new pose based on the wheel velocities
        new_pose = self.simulate_pose_from_velocities(vl, vr)

        # Extract the position components
        new_x, new_y, _ = new_pose

        # Adjust the coordinates based on the robot's radius
        adjusted_x = new_x + self.robot_radius * cos(new_pose[2])
        adjusted_y = new_y + self.robot_radius * sin(new_pose[2])

        # Convert the new position to grid coordinates
        px = int(adjusted_x / self.world_resolution)
        py = int(adjusted_y / self.world_resolution)

        # Check for collisions with obstacles in the occupancy grid
        if 0 <= px < self.map_width and 0 <= py < self.map_height:
            if self.occupancy_grid[py, px] > 0:
                # Collision detected, stop the robot
                return True

        return False

    def simulate_pose_from_velocities(self, vl, vr):
        # Simulate the new pose based on the wheel velocities
        radius = self.robot_radius
        distance = self.robot_wheel_distance

        dt = 0.1  # Time step for integration

        omega = (vr - vl) / distance
        v = (vr + vl) / 2.0

        dx = v * cos(self.current_pose['theta']) * dt
        dy = v * sin(self.current_pose['theta']) * dt
        dtheta = omega * dt

        # Simulate the new pose
        new_x = self.current_pose['x'] + dx
        new_y = self.current_pose['y'] + dy
        new_theta = self.current_pose['theta'] + dtheta

        return new_x, new_y, new_theta

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
        # Body
        self.robot_radius = robot['body']['radius']
        self.robot_height = robot['body']['height']

        # Wheels
        self.robot_wheel_distance = robot['wheels']['distance']
        self.robot_error_variance_l = robot['wheels']['error_variance_left']
        self.robot_error_variance_r = robot['wheels']['error_variance_right']

        # Laser
        self.laser_rate = robot['laser']['rate']
        self.laser_count = robot['laser']['count']
        self.laser_angle_min = robot['laser']['angle_min']
        self.laser_angle_max = robot['laser']['angle_max']
        self.laser_range_min = robot['laser']['range_min']
        self.laser_range_max = robot['laser']['range_max']
        self.laser_error_variance = robot['laser']['error_variance']
        self.laser_fail_probaability = robot['laser']['fail_probability']
    
    def publish_map(self):
        # Create an OccupancyGrid message
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'world'
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.info.resolution = self.world_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = 0.0
        map_msg.info.origin.position.y = 0.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Convert the 2D numpy array to a flattened list
        map_data = self.occupancy_grid.flatten().tolist()
        map_msg.data = map_data

        # Publish the map
        self.map_publisher.publish(map_msg)
    
    def publish_laser_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.frame_id = 'laser'
        scan_msg.header.stamp = self.get_clock().now().to_msg()

        # Get simulated laser movements
        ranges, intensities = self.simulate_laser_measurements()

        # Pushes laser variables from YAML file to message
        scan_msg.angle_min = self.laser_angle_min
        scan_msg.angle_max = self.laser_angle_max
        scan_msg.angle_increment = (self.laser_angle_max + abs(self.laser_angle_min)) / self.laser_count
        scan_msg.range_min = self.laser_range_min
        #scan_msg.range_max = self.laser_range_max
        scan_msg.range_max = self.laser_range_max
        scan_msg.scan_time = 1.0 / self.laser_rate
        scan_msg.time_increment = scan_msg.scan_time / self.laser_count

        # Simulated laser measurements
        scan_msg.ranges = ranges
        scan_msg.intensities = intensities

        # Publish laser scan
        self.scan_publisher.publish(scan_msg)

    def simulate_laser_measurements(self):
        # Fetch current robot pose
        x, y, theta = self.current_pose['x'], self.current_pose['y'], self.current_pose['theta']

        # Extract laser parameters from the robot description
        angle_min = self.laser_angle_min
        angle_max = self.laser_angle_max
        count = self.laser_count
        range_min = self.laser_range_min
        range_max = self.laser_range_max
        error_variance = self.laser_error_variance

        # Calculate the angles for each laser beam
        angles = np.linspace(
            angle_min,
            angle_max,
            count
        )

        # Simulate laser movements
        simulated_distances = []
        intensities = []
        for angle in angles:
            # Transform angle to global frame
            global_angle = theta + angle

            # Cast rays from the robot's position and simulate measurements
            distance = self.cast_ray(x, y, global_angle, range_min, range_max, error_variance)
            simulated_distances.append(distance)
            intensities.append(383.0)

        # Simulate laser measurements with random noise
        #true_distances = self.calculate_true_distances(angles)
        #noisy_distances = self.add_noise_to_distances(true_distances)
        #noisy_distances = np.random.normal(true_distances, self.laser_error_variance)
        #noisy_distances = list(noisy_distances)

        #intensities = np.full_like(noisy_distances, fill_value=383.0)
        #intensities = list(intensities)

        return simulated_distances, intensities
    
    def cast_ray(self, x, y, angle, range_min, range_max, error_variance):
        # Cast a ray from the robot's position in the specified direction
        dx = np.cos(angle)
        dy = np.sin(angle)

        # Simulate laser measurements
        for d in np.arange(range_min, range_max, 0.01):  # Adjust the step size as needed
            px = int((x + d * dx) / self.world_resolution)
            py = int((y + d * dy) / self.world_resolution)

            # Check for collisions with obstacles in the occupancy grid
            if 0 <= px < self.map_width and 0 <= py < self.map_height:
                if self.occupancy_grid[py, px] > 0:
                    # Collision detected
                    return d + np.random.normal(0, error_variance)

        # No collision, return maximum range
        return range_max + np.random.normal(0, error_variance)
    
    def calculate_true_distances(self, angles):
        # Calculate true distances to obstacle based on the robot's pose and world map
        #return np.ones_like(angles) * self.laser_range_max
        return np.ones_like(angles) * self.laser_range_max
    
    def add_noise_to_distances(self, true_distances):
        # Add random noise to laser measurements based on error characteristics
        return true_distances + np.random.normal(0.0, self.laser_error_variance, len(true_distances))
    
    def parse_map_string(self):
        lines = self.world_map.strip().split('\n')  # Separate lines
        rows = [list(line.strip()) for line in reversed(lines)]

        map_h = 0
        map_w = 0

        # Convert map symbols to numeric values (0 for free space, 100 for obstacles)
        occupancy_grid = np.zeros((len(rows), len(rows[0])), dtype=np.int8)
        for i, row in enumerate(rows):
            map_h += 1
            map_w = 0
            for j, symbol in enumerate(row):
                map_w += 1
                if symbol == '.':
                    occupancy_grid[i, j] = 0  # Free space
                elif symbol == '#':
                    occupancy_grid[i, j] = 100  # Obstacle

        self.map_width = map_w
        self.map_height = map_h

        return occupancy_grid

    def load_world(self, file_name):
        with open(file_name, 'r') as f:
            world = yaml.safe_load(f)
        world['urdf'] = self.fetch_world_info(world)
    
    def fetch_world_info(self, world):
        self.world_resolution = world['resolution']
        self.world_initial_pose = world['initial_pose']
        self.world_map = world['map']


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