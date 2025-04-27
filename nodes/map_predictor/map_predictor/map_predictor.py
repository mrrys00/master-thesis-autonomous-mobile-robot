import os
import json
from datetime import datetime

from copy import deepcopy
from json import load
import os
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from scipy.ndimage import label

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

VAL_INACCESSIBLE = 200

VAL_UNKNOWN = -1
VAL_FREE = 0
VAL_OCCUPIED = 100

# TODO - add description for every method

class MapPredictorNode(Node):
    def __init__(self):
        super().__init__('map_predictor')
        
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
        
        # Create a timer to periodically process the latest map
        self.create_timer(10.0, self.process_latest_map)
        
        self.get_logger().info("Map Predictor Node has been started.")
        
        self.latest_map: OccupancyGrid = None


    def map_callback(self, msg: OccupancyGrid):
        """
        Callback function for the /map topic. This function processes the received map data and saves it to a JSON file.
        It also updates the latest_map attribute with the current map data.

        Args:
            msg (OccupancyGrid): Currenty received map data.
        """
        try:
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
            
            self.latest_map = deepcopy(msg)

        except Exception as e:
            self.get_logger().error(f"Failed to process and save map data: {e}")

    def process_latest_map(self):
        """
        Process the latest map data and calculate the remaining space in the map.
        """
        try:
            self.get_logger().info("Processing the latest map data.")
            # TODO - measure processing time
            
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
            - VAL_INACCESSIBLE (200): The cell is inaccessible.
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

    def fill_enclosed_unknowns_v2(self, grid: np.ndarray) -> np.ndarray:
        """
        Fill enclosed unknown regions in the grid with VAL_INACCESSIBLE.
        This function uses connected components to identify regions of unknown cells (-1) and fills them with VAL_INACCESSIBLE (200)
        if they are not connected to any free cells (0) or touch the boundary of the grid.

        Args:
            grid (np.ndarray): The occupancy grid of the map.

        Returns:
            np.ndarray: The modified grid with enclosed unknown regions filled with VAL_INACCESSIBLE.
        """
        filled_grid = grid.copy()
        height, width = grid.shape
        
        # Identify all -1 regions using connected components
        labeled_grid, num_features = label(grid == -1)
        
        # Find which regions are connected to 1 or touch the boundary
        invalid_regions = set()
        for i in range(height):
            for j in range(width):
                if labeled_grid[i, j] > 0:
                    # If the region is adjacent to 1, mark it as invalid
                    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nx, ny = i + dx, j + dy
                        if 0 <= nx < height and 0 <= ny < width and grid[nx, ny] == VAL_FREE:
                            invalid_regions.add(labeled_grid[i, j])
                    # If the region touches the boundary, also mark it as invalid
                    if i == 0 or i == height - 1 or j == 0 or j == width - 1:
                        invalid_regions.add(labeled_grid[i, j])
        
        # Convert enclosed unknown regions (only surrounded by VAL_OCCUPIED) to VAL_INACCESSIBLE
        for region_id in range(1, num_features + 1):
            if region_id not in invalid_regions:
                filled_grid[labeled_grid == region_id] = VAL_INACCESSIBLE
        
        return filled_grid
    
    def is_fully_enclosed(self, grid: np.ndarray) -> bool:
        """
        Check if the grid is fully enclosed by VAL_OCCUPIED cells.
        This function uses a flood fill algorithm to check if there are any free cells (0) that are reachable from the boundary of the grid.
        If any free cell is reachable from the boundary, the grid is not fully enclosed.
        Otherwise, it is fully enclosed.

        Args:
            grid (np.ndarray): The occupancy grid of the map.

        Returns:
            bool: True if the grid is fully enclosed, False otherwise.
        """
        height, width = grid.shape
        visited = np.zeros_like(grid, dtype=bool)
        queue = []
        
        # Start flood fill from the borders where 0 or -1 exists
        for i in range(height):
            if grid[i, 0] in (0, -1):
                queue.append((i, 0))
            if grid[i, width - 1] in (0, -1):
                queue.append((i, width - 1))
        for j in range(width):
            if grid[0, j] in (0, -1):
                queue.append((0, j))
            if grid[height - 1, j] in (0, -1):
                queue.append((height - 1, j))
        
        while queue:
            x, y = queue.pop()
            if visited[x, y] or grid[x, y] == VAL_OCCUPIED:
                continue
            visited[x, y] = True
            
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < height and 0 <= ny < width and not visited[nx, ny] and grid[nx, ny] in (0, -1):
                    queue.append((nx, ny))
        
        # If any 0 is reachable from the boundary, it is not enclosed
        return not np.any((grid == VAL_FREE) & visited)

    def fill_outside_with_val_inaccessible(self, grid: np.ndarray) -> np.ndarray:
        """
        Fill the outside of the grid with VAL_INACCESSIBLE (200).
        This function uses a flood fill algorithm to identify all external areas of the grid that are not occupied (VAL_OCCUPIED).

        Args:
            grid (np.ndarray): The occupancy grid of the map.

        Returns:
            np.ndarray: The modified grid with external areas filled with VAL_INACCESSIBLE.
        """
        filled_grid = grid.copy()
        height, width = grid.shape
        
        # Identify all external areas using flood fill from edges
        visited = np.zeros_like(grid, dtype=bool)
        queue = []
        
        for i in range(height):
            if grid[i, 0] != VAL_OCCUPIED:
                queue.append((i, 0))
            if grid[i, width - 1] != VAL_OCCUPIED:
                queue.append((i, width - 1))
        for j in range(width):
            if grid[0, j] != VAL_OCCUPIED:
                queue.append((0, j))
            if grid[height - 1, j] != VAL_OCCUPIED:
                queue.append((height - 1, j))
        
        while queue:
            x, y = queue.pop()
            if visited[x, y] or grid[x, y] == VAL_OCCUPIED:
                continue
            visited[x, y] = True
            
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < height and 0 <= ny < width and not visited[nx, ny] and grid[nx, ny] != VAL_OCCUPIED:
                    queue.append((nx, ny))
        
        # Any non-VAL_OCCUPIED space that is not visited is fully enclosed, so fill the outside with VAL_OCCUPIED
        filled_grid[visited] = VAL_INACCESSIBLE
        
        return filled_grid

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MapPredictorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Map Predictor Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
