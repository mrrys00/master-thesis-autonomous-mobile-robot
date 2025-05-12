import os
import json
from datetime import datetime

from copy import deepcopy
from json import load
import os
import numpy as np
# import matplotlib.pyplot as plt
# from sklearn.cluster import DBSCAN
# from scipy.ndimage import label
from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry

VAL_UNKNOWN = -1
VAL_FREE = 0
VAL_OCCUPIED = 100
VAL_INACCESSIBLE = 101

VAL_CURR_POSITION = 200
VAL_NEXT_GOAL = 201

VAL_ESTIMATED_WALL = 250

RESOLUTION = 0.05 # TODO read resolution from config file
DEBUG = False # TODO read debug from config file

# TODO - rewrite to explore and catch time to target

class TimePredictorNode(Node):
    def __init__(self):
        super().__init__('time_predictor')
        
        self.declare_parameter('output_directory', 'messages')
        
        self.output_directory = self.get_parameter('output_directory').get_parameter_value().string_value
        
        # Ensure output directory exists
        if not os.path.exists(self.output_directory):
            os.makedirs(self.output_directory)

        # Subscribe to the /map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Subscribe to the /map topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Create navigation client
        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Create a timer to periodically process the latest map
        self.create_timer(10.0, self.process_latest_map)
        
        self.get_logger().info("Map Predictor Node has been started.")
        
        self.latest_map: OccupancyGrid | dict = None
        self.latest_odom: Odometry | dict = None


    def map_callback(self, msg: OccupancyGrid):
        """
        Callback function for the /map topic. This function processes the received map data and saves it to a JSON file.
        It also updates the latest_map attribute with the current map data.

        Args:
            msg (OccupancyGrid): Currenty received map data.
        """
        self.latest_map = deepcopy(msg)
        
        try:
            if not DEBUG:
                # temporary not to save json files
                raise Exception("DEBUG is not set to True.")

            map_data = {
                'header': {
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': msg.header.frame_id
                },
                'info': {
                    'map_load_time': {
                        'sec': msg.info.map_load_time.sec,
                        'nanosec': msg.info.map_load_time.nanosec
                    },
                    'resolution': msg.info.resolution,
                    'width': msg.info.width,
                    'height': msg.info.height,
                    'origin': {
                        'position': {
                            'x': msg.info.origin.position.x,
                            'y': msg.info.origin.position.y,
                            'z': msg.info.origin.position.z
                        },
                        'orientation': {
                            'x': msg.info.origin.orientation.x,
                            'y': msg.info.origin.orientation.y,
                            'z': msg.info.origin.orientation.z,
                            'w': msg.info.origin.orientation.w
                        }
                    }
                },
                'data': list(msg.data)
            }

            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filename = os.path.join(self.output_directory, f'map_{timestamp}.json')

            with open(filename, 'w') as json_file:
                json.dump(map_data, json_file, indent=4)

                self.get_logger().info(f"Saved map data to {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to process and save map data: {e}")

    def odom_callback(self, msg: Odometry):
        """_summary_

        Args:
            msg (Odometry): _description_
        """
        
        self.latest_odom = deepcopy(msg)
    
# ========== Map Processing Functions ==========

    def get_reachable_mask(self,grid: np.ndarray, position: tuple[int, int]) -> np.ndarray[tuple[()], np.dtype]:
        h, w = grid.shape
        visited = np.zeros_like(grid, dtype=bool)
        q = deque([position])
        visited[position[1], position[0]] = True
        
        while q:
            x, y = q.popleft()
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    if not visited[ny, nx] and grid[ny, nx] in [0, -1]:
                        visited[ny, nx] = True
                        q.append((nx, ny))

        return visited

    def fill_outside_with_val_inaccessible(self, grid: np.ndarray, position: tuple[int, int]) -> np.ndarray[tuple[()], np.dtype]:
        reachable = self.get_reachable_mask(grid, position)
        grid[(grid == -1) & (~reachable)] = VAL_INACCESSIBLE
        
        return grid

    def is_fully_enclosed(self, grid: np.ndarray, position: tuple[int, int]) -> bool:
        reachable = self.get_reachable_mask(grid, position)
        unknown_mask = (grid == -1)
        
        return not np.any(reachable & unknown_mask)

    def fill_enclosed_unknowns_v2(self, grid: np.ndarray, position: tuple[int, int]) -> np.ndarray[tuple[()], np.dtype]:
        reachable = self.get_reachable_mask(grid, position)
        unknown = (grid == -1)
        enclosed = unknown & (~reachable)
        grid[enclosed] = VAL_INACCESSIBLE
        
        return grid

    def fill_boundary_unknowns(self, grid: np.ndarray, position: tuple[int, int]) -> np.ndarray[tuple[()], np.dtype]:
        reachable = self.get_reachable_mask(grid, position)
        h, w = grid.shape
        for y in range(h):
            for x in range(w):
                if grid[y, x] == -1 and reachable[y, x]:
                    neighbors = [(x+dx, y+dy) for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]]
                    if any(0 <= nx < w and 0 <= ny < h and grid[ny, nx] == 0 for nx, ny in neighbors):
                        grid[y, x] = 0
        
        return grid

    def fill_boundary_gaps(self, grid: np.ndarray, position: tuple[int, int]) -> np.ndarray[tuple[()], np.dtype]:
        reachable = self.get_reachable_mask(grid, position)
        h, w = grid.shape
        for y in range(h):
            for x in range(w):
                if grid[y, x] == -1 and reachable[y, x]:
                    neighbors = [(x+dx, y+dy) for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]]
                    free_neighbors = [grid[ny, nx] == 0 for nx, ny in neighbors if 0 <= nx < w and 0 <= ny < h]
                    if sum(free_neighbors) >= 2:
                        grid[y, x] = 0
        
        return grid

    def process_latest_map(self):
        """
        Process the latest map data and calculate the remaining space in the map.
        """
        try:
            self.get_logger().info("Processing the latest map data.")
            
            _data, _height, _width = self.latest_map.data, self.latest_map.info.height, self.latest_map.info.width
            _grid = np.array(_data).reshape((_height, _width))
            
            filled_grid = self.fill_enclosed_unknowns_v2(_grid)
            fully_enclised = self.is_fully_enclosed(filled_grid)
            if fully_enclised: filled_grid = self.fill_outside_with_val_inaccessible(filled_grid)
            occupied,\
            free,\
            inaccessible,\
            unknown,\
            explored_percent = self.calculate_remaining_space(filled_grid)
            
            self.get_logger().info("Processed data:\n" +
            f"\tOccupied: {occupied}\n" + \
            f"\tFree: {free}\n" + \
            f"\tInaccessible: {inaccessible}\n" + \
            f"\tUnknown: {unknown}\n" + \
            f"\tExplored: {100*explored_percent:.8f}%")
        except:
            self.get_logger().warn("No map data available to process.")
            
    def calculate_remaining_space(self, grid: np.ndarray[tuple[()], np.dtype]) -> tuple:
        """
        Calculate the remaining space in the map.
        This function counts the number of occupied, free, inaccessible, and unknown cells in the grid.
        It also calculates the percentage of explored space in the grid.

        Args:
            grid (np.ndarray[tuple[): The occupancy grid of the map.
        The grid is a 2D numpy array where each cell can have one of the following values:
            - VAL_OCCUPIED (100): The cell is occupied.
            - VAL_FREE (0): The cell is free.
            - VAL_INACCESSIBLE (101): The cell is inaccessible.
            - VAL_UNKNOWN (-1): The cell is unknown.
        The grid is represented as a numpy array of shape (height, width).

        Returns:
            tuple: occupied, free, inaccessible, unknown, explored_percent
        """
        _explored_num = np.count_nonzero(grid==VAL_OCCUPIED) + np.count_nonzero(grid==VAL_FREE) + np.count_nonzero(grid==VAL_INACCESSIBLE)
        _height, _width = grid.shape
        _explored_percent = _explored_num / (_width * _height)
        
        return np.count_nonzero(grid==VAL_OCCUPIED),\
            np.count_nonzero(grid==VAL_FREE),\
            np.count_nonzero(grid==VAL_INACCESSIBLE),\
            np.count_nonzero(VAL_UNKNOWN),\
            _explored_percent

# ========== Exploration Functions ==========

    def get_position(self, x: float, y: float, origin_x: float = 0.0, origin_y: float = 0.0) -> tuple[int, int]:
        """Get position in pixels from real-world coordinates.

        Args:
            x (float), y (float): Real-world coordinates to mark.
            origin_x (float, optional), origin_y (float, optional): Real-world coordinates of the map's origin (bottom-left corner). Defaults to 0.0.

        Returns:
            tuple: position x and y in pixels
        """
        
        return int((x - origin_x) / RESOLUTION), int((y - origin_y) / RESOLUTION)

    # def mark_position_v2(
    #     self, 
    #     grid: np.ndarray,
    #     x: int,
    #     y: int,
    #     color: int = VAL_CURR_POSITION
    #     ) -> np.ndarray:
    #     """
    #     Marks a position on the grid based on real-world coordinates and map origin.

    #     Args:
    #         grid: 2D numpy array representing the map.
    #         x, y: grid coordinates.

    #     Returns:
    #         Modified grid with the position marked.
    #     """
    #     height, width = grid.shape

    #     print(f"Marking position at world coordinates: ({x}, {y})")

    #     if 0 <= x < width and 0 <= y < height:
    #         grid[y, x] = color
    #     else:
    #         print("Warning: Position out of grid bounds.")

    #     return grid

    def explore_nearest_unknown(
        self,
        grid: np.ndarray,
        x: float,
        y: float,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        resolution: float = 1.0
    ) -> tuple[float | None, float | None, int | None, int | None]:
        """
        Navigate to the nearest unknown (-1) cell.

        Parameters:
            grid (np.ndarray): Occupancy grid (2D)
            x (float): Robot real-world x position
            y (float): Robot real-world y position
            origin_x (float): Map origin x in real-world coordinates
            origin_y (float): Map origin y in real-world coordinates
            resolution (float): Map resolution (m/cell)

        Returns:
            Tuple of real-world coordinates (x, y) and grid coordinates (x, y) or None if no unknowns found
        """
        height, width = grid.shape

        # Convert real-world position to grid indices
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        # Validate bounds
        if not (0 <= grid_x < width and 0 <= grid_y < height):
            raise ValueError("Robot position is out of map bounds")

        # Find all unknown cells
        unknown_indices = np.argwhere(grid == -1)
        if unknown_indices.size == 0:
            return None, None, None, None

        # Compute distances
        distances = np.linalg.norm(unknown_indices - np.array([grid_y, grid_x]), axis=1)
        nearest_idx = unknown_indices[np.argmin(distances)]

        # Convert grid indices back to real-world coordinates
        target_real_x = origin_x + nearest_idx[1] * resolution + resolution / 2
        target_real_y = origin_y + nearest_idx[0] * resolution + resolution / 2

        return (target_real_x, target_real_y, nearest_idx[1], nearest_idx[0])

    def method_to_process_and_catch_goal(self, goal_x: float, goal_y: float) -> tuple[float | None, float | None]:
        """
        Process and catch goal position
        """
        # TODO - write this function

    def send_navigation_goal(self, goal_pose):
        """
        Send navigation goal
        """
        self.get_logger().info(f'Sending navigation goal as xyzw: ({goal_pose.pose.position.x}, {goal_pose.pose.position.y}, {goal_pose.pose.orientation.z}, {goal_pose.pose.orientation.w})')

        # prepare goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.navigation_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TimePredictorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Time Predictor Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
