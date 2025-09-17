import os
import json
from datetime import datetime

from copy import deepcopy
from json import load
import os
import numpy as np
from math import sin, cos, pi
# import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
# from scipy.ndimage import label
from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header


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
        
        self.create_timer(10.0, self.explore_by_frontiers)
        
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
        """
        Callback function for handling Odometry messages.
        This function is triggered whenever a new Odometry message is received.
        It creates a deep copy of the message and stores it in the `latest_odom` attribute.
        Args:
            msg (Odometry): The incoming Odometry message.
        """
        
        self.latest_odom = deepcopy(msg)
    
# ========== Map Processing Functions ==========

    def get_reachable_mask(self,grid: np.ndarray, position: tuple[int, int]) -> np.ndarray[tuple[()], np.dtype]:
        """
        Computes a mask of reachable cells in a grid starting from a given position.
        This function performs a breadth-first search (BFS) to determine which cells 
        in the grid can be reached from the starting position. A cell is considered 
        reachable if it contains a value of 0 or -1.
        Args:
            grid (np.ndarray): A 2D numpy array representing the grid. Each cell 
                contains an integer value.
            position (tuple[int, int]): A tuple (x, y) representing the starting 
                position in the grid. `x` is the column index, and `y` is the row index.
        Returns:
            np.ndarray: A 2D boolean numpy array of the same shape as `grid`, where 
                `True` indicates that the corresponding cell is reachable, and 
                `False` indicates that it is not.
        """
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
        """
        Modifies the input grid by filling cells that are inaccessible and have a value of -1 
        with a predefined constant value `VAL_INACCESSIBLE`.
        Args:
            grid (np.ndarray): A 2D numpy array representing the grid.
            position (tuple[int, int]): A tuple representing the starting position 
                (row, column) in the grid.
        Returns:
            np.ndarray: The modified grid with inaccessible cells updated.
        """
        reachable = self.get_reachable_mask(grid, position)
        grid[(grid == -1) & (~reachable)] = VAL_INACCESSIBLE
        
        return grid

    def is_fully_enclosed(self, grid: np.ndarray, position: tuple[int, int]) -> bool:
        """
        Determines if a given position in a grid is fully enclosed.
        A position is considered fully enclosed if there are no unknown cells 
        (cells with a value of -1) that are reachable from the given position.
        Args:
            grid (np.ndarray): A 2D numpy array representing the grid. 
                               Cells with a value of -1 are considered unknown.
            position (tuple[int, int]): The (row, column) coordinates of the position to check.
        Returns:
            bool: True if the position is fully enclosed, False otherwise.
        """
        reachable = self.get_reachable_mask(grid, position)
        unknown_mask = (grid == -1)
        
        return not np.any(reachable & unknown_mask)

    def fill_enclosed_unknowns_v2(self, grid: np.ndarray, position: tuple[int, int]) -> np.ndarray[tuple[()], np.dtype]:
        """
        Fills enclosed unknown cells in a grid with a specific value.
        This method identifies cells in the grid that are marked as unknown (value -1) 
        and are not reachable from a given starting position. These enclosed unknown 
        cells are then updated with the value `VAL_INACCESSIBLE`.
        Args:
            grid (np.ndarray): A 2D numpy array representing the grid. Unknown cells 
                are represented by the value -1.
            position (tuple[int, int]): A tuple representing the starting position 
                (row, column) in the grid.
        Returns:
            np.ndarray: The updated grid with enclosed unknown cells filled with 
            `VAL_INACCESSIBLE`.
        """
        reachable = self.get_reachable_mask(grid, position)
        unknown = (grid == -1)
        enclosed = unknown & (~reachable)
        grid[enclosed] = VAL_INACCESSIBLE
        
        return grid

    def fill_boundary_unknowns(self, grid: np.ndarray, position: tuple[int, int]) -> np.ndarray[tuple[()], np.dtype]:
        """
        Fills the boundary cells of a grid that are marked as unknown (-1) with 0 if they are reachable
        and adjacent to a cell with a value of 0.
        Args:
            grid (np.ndarray): A 2D numpy array representing the grid, where -1 indicates unknown cells,
                               0 indicates empty cells, and other values represent obstacles or other states.
            position (tuple[int, int]): The starting position (x, y) in the grid from which reachability is determined.
        Returns:
            np.ndarray: The updated grid with boundary unknown cells filled with 0 where applicable.
        Notes:
            - The method uses the `get_reachable_mask` function to determine which cells are reachable
              from the given position.
            - A cell is considered a boundary unknown if it is marked as -1 and is adjacent to at least
              one cell with a value of 0.
            - The grid is updated in-place and also returned for convenience.
        """
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
        """
        Fills gaps in the grid boundary by marking certain unreachable cells as free (value 0) 
        if they are adjacent to at least two free cells and are reachable from the given position.
        Args:
            grid (np.ndarray): A 2D numpy array representing the grid, where:
                -1 indicates an unreachable cell,
                 0 indicates a free cell,
                 other values may represent obstacles or other states.
            position (tuple[int, int]): The starting position (x, y) in the grid from which 
                reachability is determined.
        Returns:
            np.ndarray: The updated grid with certain boundary gaps filled.
        """
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


    def explore_nearest_frontier(
        self,
        grid: np.ndarray,
        x: float,
        y: float,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        resolution: float = RESOLUTION
    ) -> tuple[float | None, float | None, int | None, int | None]:
        """
        Navigate to the most promising unknown cell to escape enclosure.

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

        # Convert world position to grid coordinates
        start_x = int((x - origin_x) / resolution)
        start_y = int((y - origin_y) / resolution)

        if not (0 <= start_x < width and 0 <= start_y < height):
            self.get_logger().warn("Robot start position is out of bounds.")
            return None, None, None, None

        visited = np.zeros_like(grid, dtype=bool)
        queue = deque([(start_x, start_y)])
        visited[start_y, start_x] = True

        while queue:
            cx, cy = queue.popleft()

            # If current free cell is adjacent to unknown, return it
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    if grid[ny, nx] == VAL_UNKNOWN:
                        # This cell (cx, cy) is on the boundary of free and unknown
                        real_x = origin_x + cx * resolution + resolution / 2
                        real_y = origin_y + cy * resolution + resolution / 2
                        self.get_logger().info(f"Targeting frontier cell at: ({cx}, {cy})")
                        return real_x, real_y, cx, cy

            # Continue BFS
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    if not visited[ny, nx] and grid[ny, nx] == VAL_FREE:
                        visited[ny, nx] = True
                        queue.append((nx, ny))

        self.get_logger().info("No reachable unknown boundaries found.")
        return None, None, None, None
    

    def explore_nearest_frontier_v2(
        self,
        grid: np.ndarray,
        x: float,
        y: float,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        resolution: float = RESOLUTION
    ) -> tuple[float | None, float | None, int | None, int | None]:
        """
        Find the most valuable frontier cell based on:
        1. Largest frontier (connected wall cells adjacent to unknowns)
        2. Highest surrounding unknown density

        Returns:
            real_x, real_y, grid_x, grid_y — best cell to navigate to
        """
        height, width = grid.shape
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        if not (0 <= grid_x < width and 0 <= grid_y < height):
            self.get_logger().warn("Robot start position is out of bounds.")
            return None, None, None, None

        visited = np.zeros_like(grid, dtype=bool)
        frontier_groups = []

        def is_frontier(x, y):
            if grid[y, x] != VAL_OCCUPIED:
                return False
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height and grid[ny, nx] == VAL_UNKNOWN:
                    return True
            return False

        # 1. Identify all frontier groups
        for y in range(height):
            for x in range(width):
                if is_frontier(x, y) and not visited[y, x]:
                    group = []
                    q = deque([(x, y)])
                    visited[y, x] = True
                    while q:
                        cx, cy = q.popleft()
                        group.append((cx, cy))
                        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                            nx, ny = cx + dx, cy + dy
                            if (0 <= nx < width and 0 <= ny < height and
                                not visited[ny, nx] and is_frontier(nx, ny)):
                                visited[ny, nx] = True
                                q.append((nx, ny))
                    if group:
                        frontier_groups.append(group)

        if not frontier_groups:
            self.get_logger().info("No frontier groups found.")
            return None, None, None, None

        # 2. Evaluate each group
        best_score = -float("inf")
        best_point = None
        for group in frontier_groups:
            group_size = len(group)
            unknown_score = 0
            group_center = group[0]

            for gx, gy in group:
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < width and 0 <= ny < height and grid[ny, nx] == VAL_UNKNOWN:
                            unknown_score += 1

            # Prefer large wall frontiers, then high unknown density
            score = group_size * 10 + unknown_score

            if score > best_score:
                best_score = score
                best_point = group_center

        if best_point:
            real_x = origin_x + best_point[0] * resolution + resolution / 2
            real_y = origin_y + best_point[1] * resolution + resolution / 2
            self.get_logger().info(f"Selected frontier at ({best_point[0]}, {best_point[1]}) with score {best_score}")
            return real_x, real_y, best_point[0], best_point[1]

        return None, None, None, None

    def explore_wall_follow_ccw(
        self,
        grid: np.ndarray,
        x: float,
        y: float,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        resolution: float = RESOLUTION
    ) -> tuple[float | None, float | None, int | None, int | None]:
        """
        Navigate to the next unknown cell by:
        1. Navigating to the nearest wall (100)
        2. Following wall counter-clockwise to find unknowns

        Returns:
            real_x, real_y, grid_x, grid_y
        """

        height, width = grid.shape
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        if not (0 <= grid_x < width and 0 <= grid_y < height):
            self.get_logger().warn("Robot start position is out of bounds.")
            return None, None, None, None

        # Step 1: Find the nearest wall (100)
        visited = np.zeros_like(grid, dtype=bool)
        q = deque([(grid_x, grid_y)])
        visited[grid_y, grid_x] = True
        wall_target = None

        while q:
            cx, cy = q.popleft()
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    if grid[ny, nx] == VAL_OCCUPIED:
                        wall_target = (cx, cy)
                        break
                    if not visited[ny, nx] and grid[ny, nx] == VAL_FREE:
                        visited[ny, nx] = True
                        q.append((nx, ny))
            if wall_target:
                break

        if wall_target is None:
            self.get_logger().info("No wall found.")
            return None, None, None, None

        # Step 2: Wall-following CCW (simple loop around the wall)
        def get_neighbors_ccw(x, y):
            # Order: up, left, down, right (simulates CCW scan)
            return [(x, y - 1), (x - 1, y), (x, y + 1), (x + 1, y)]

        visited_wall = set()
        q = deque([wall_target])
        visited_wall.add(wall_target)

        while q:
            cx, cy = q.popleft()
            for nx, ny in get_neighbors_ccw(cx, cy):
                if 0 <= nx < width and 0 <= ny < height:
                    if grid[ny, nx] == VAL_UNKNOWN:
                        # Found unknown along wall
                        real_x = origin_x + cx * resolution + resolution / 2
                        real_y = origin_y + cy * resolution + resolution / 2
                        return real_x, real_y, cx, cy
                    elif grid[ny, nx] == VAL_FREE and (nx, ny) not in visited_wall:
                        # Continue hugging wall
                        wall_adjacent = any(
                            0 <= ax < width and 0 <= ay < height and grid[ay, ax] == VAL_OCCUPIED
                            for ax, ay in get_neighbors_ccw(nx, ny)
                        )
                        if wall_adjacent:
                            visited_wall.add((nx, ny))
                            q.append((nx, ny))

        self.get_logger().info("No unknown found along wall.")
        return None, None, None, None

    def explore_by_frontiers(self):
        """
        Process and catch goal position
        """
        # TODO
        
        goal_x, goal_y, _, _ = self.explore_nearest_frontier_v2(
            np.array(self.latest_map.data).reshape((self.latest_map.info.height, self.latest_map.info.width)),
            self.latest_odom.pose.pose.position.x,
            self.latest_odom.pose.pose.position.y,
            self.latest_map.info.origin.position.x,
            self.latest_map.info.origin.position.y,
            RESOLUTION
        )
        
        new_cords = self.prepare_pose_stamped(
            goal_x, goal_y,
            self.latest_odom.pose.pose.orientation.z,
            self.latest_odom.pose.pose.orientation.w)
        
        self.send_navigation_goal(new_cords)
        
    def prepare_pose_stamped(self, _x: float, _y: float, _ori_z: float, _ori_w: float) -> PoseStamped:
        """
        Prepare PoseStamped message structure from distance and actual position
        """

        goal_pose = PoseStamped()
        goal_pose.header = Header()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = _x
        goal_pose.pose.position.y = _y
        # goal_pose.pose.orientation.z = sin(_theta / 2.0)
        # goal_pose.pose.orientation.w = cos(_theta / 2.0)
        goal_pose.pose.orientation.z = _ori_z
        goal_pose.pose.orientation.w = _ori_w
        
        return goal_pose

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