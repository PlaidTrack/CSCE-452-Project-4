import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from math import cos, sin
import numpy as np

class SimulateMap(Node):
    def __init__(self):
        super().__init__('simulate_map')

        input_world = '/root/ros2_project4/src/project4/world_data/brick.world'

        self.world_resolution = 0.0
        self.world_initial_pose = [0.0, 0.0, 0.0]
        self.world_map = ""

        self.load_world(input_world)

        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)
        
    def publish_map(self):
        # Parse the map string and convert it to an occupancy grid
        occupancy_grid = self.parse_map_string()

        # Create an OccupancyGrid message
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'odom'
        map_msg.info.resolution = self.world_resolution
        map_msg.info.origin.position.x = self.world_initial_pose[0]
        map_msg.info.origin.position.y = self.world_initial_pose[1]
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Convert the 2D numpy array to a flattened list
        map_data = occupancy_grid.flatten().tolist()
        map_msg.data = map_data

        # Publish the map
        self.map_publisher.publish(map_msg)
    
    def parse_map_string(self):
        lines = self.world_map.strip().split('\n')  # Separate lines
        rows = [list(line.strip()) for line in lines]

        # Convert map symbols to numeric values (0 for free space, 100 for obstacles)
        occupancy_grid = np.zeros((len(rows), len(rows[0])), dtype=np.int8)
        for i, row in enumerate(rows):
            for j, symbol in enumerate(row):
                if symbol == '.':
                    occupancy_grid[i, j] = 0  # Free space
                elif symbol == '#':
                    occupancy_grid[i, j] = 100  # Obstacle

        return occupancy_grid

    def load_world(self, file_name):
        with open(file_name, 'r') as f:
            world = yaml.safe_load(f)
        world['urdf'] = self.fetch_world_info(world)
        print(self.world_resolution)
        print(self.world_initial_pose)
        print(self.world_map)

    def fetch_world_info(self, world):
        self.world_resolution = world['resolution']
        self.world_initial_pose = world['initial_pose']
        self.world_map = world['map']

def main():
    rclpy.init()
    simulate_map = SimulateMap()
    rclpy.spin(simulate_map)
    rclpy.shutdown()


if __name__ == '__main__':
    main()