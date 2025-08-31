import rclpy

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import heapq, math, random, yaml
import json
import logging
import numpy as np
import os
import scipy.interpolate as si
import sys, threading, time

from collections import deque
from scipy import ndimage
from datetime import datetime


pathGlobal = 0

VAL_FRONTIER = 2

VAL_UNKNOWN = -1
VAL_FREE = 0
VAL_OCCUPIED = 100
VAL_INACCESSIBLE = 101

VAL_CURR_POSITION = 102
VAL_NEXT_GOAL = 103

VAL_LARGEST_OBSTACLE = 104

VAL_OCCUPIED_MATRIX = 1.0


class BoundaryExploration(Node):
    def __init__(self):
        super().__init__('boundary_finder_node')
        
        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('lookahead_distance', 0.5),
                ('speed', 0.15),
                ('expansion_size', 3),
                ('target_error', 0.1),
                ('robot_r', 0.18),
                ('resolution', 0.05),
                ('map_dump', True),
                ('output_directory', 'messages'),
                ('log_level', 'INFO')
            ])
        
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.expansion_size = self.get_parameter('expansion_size').get_parameter_value().integer_value
        self.target_error = self.get_parameter('target_error').get_parameter_value().double_value
        self.robot_r = self.get_parameter('robot_r').get_parameter_value().double_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.map_dump = self.get_parameter('map_dump').get_parameter_value().bool_value
        self.output_directory = self.get_parameter('output_directory').get_parameter_value().string_value
        
        # Set log level
        match self.get_parameter('log_level').get_parameter_value().string_value.upper():
            case 'DEBUG':
                self.get_logger().set_level(logging.DEBUG)
            case 'INFO':
                self.get_logger().set_level(logging.INFO)
            case 'WARN' | 'WARNING':
                self.get_logger().set_level(logging.WARN)
            case 'ERROR':
                self.get_logger().set_level(logging.ERROR)
            case 'FATAL' | 'CRITICAL':
                self.get_logger().set_level(logging.CRITICAL)
        
        # Create output directory if it doesn't exist
        if self.map_dump and not os.path.exists(self.output_directory):
            os.makedirs(self.output_directory)
        
        # Initialize subscriptions and publishers
        self.subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize variables
        self.latest_map: OccupancyGrid | dict = None
        self.latest_odom: Odometry | dict = None

        self.target: bool = True
        
        self.scan_data: LaserScan | None
        self.scan: list[float] | any
        
        self.map_data: OccupancyGrid | None
        self.originX: float | any
        self.originY: float | any
        self.width: int | any
        self.height: int | any
        self.map_occupancy_data: list[int] | any
        
        self.odom_data: Odometry | None
        self.x: float | any
        self.y: float | any
        self.yaw: float | any
        
        self.path: list[tuple[any | list[list | any], any | list[list | any]]]  |  np.ndarray[tuple[int, ...], np.dtype]  |  int
        
        self.i: int
        self.c: int
        self.r: int
        self.t: threading.Timer
        
        self.last_frontier_groups: dict[int, list[tuple[int, int]]] = {}
        self.largest_obstacle: list[tuple[int, int]] = []
        
        self.just_reached_goal: bool = False
        
        self.stop_exploration: bool = False
        
        threading.Thread(target=self.exp).start()
        
        self.get_logger().info("Exploration node initialized.")
        self.get_logger().debug(f"Parameters: lookahead_distance={self.lookahead_distance}, speed={self.speed}, expansion_size={self.expansion_size}, target_error={self.target_error}, robot_r={self.robot_r}, resolution={self.resolution}, map_dump={self.map_dump}, output_directory='{self.output_directory}'")
        
    def exp(self):
        """
        Main exploration loop.
        """
        twist = Twist()
        
        while True:
            if not hasattr(self,'map_data') or not hasattr(self,'odom_data') or not hasattr(self,'scan_data'):
                time.sleep(0.1)
                continue

            if self.target == True:
                if isinstance(pathGlobal, int) and pathGlobal == 0:
                    column = int((self.x - self.originX)/self.resolution)
                    row = int((self.y- self.originY)/self.resolution)
                    self.exploration(self.map_occupancy_data, 
                        self.width, 
                        self.height, 
                        self.resolution, 
                        column, 
                        row, 
                        self.originX, 
                        self.originY)
                    self.path = pathGlobal
                else:
                    self.path = pathGlobal
                if isinstance(self.path, int) and self.path == -1:
                    self.get_logger().info("Exploration completed. Stopping the robot.")
                    sys.exit()

                self.c = int((self.path[-1][0] - self.originX)/self.resolution) 
                self.r = int((self.path[-1][1] - self.originY)/self.resolution) 
                self.target = False
                self.i = 0
                self.get_logger().info(f"New target set: ({self.path[-1][0]:.2f}, {self.path[-1][1]:.2f})")
                
                self.mark_frontiers_and_goal_on_dump_map()
                
                t = self.pathLength(self.path)/self.speed
                t = t - 0.2
                self.t = threading.Timer(t, self.target_callback)
                self.t.start()
            else:
                v , w = self.localControl(self.scan)
                if v == None:
                    v, w, self.i = self.pure_pursuit(
                        self.x,
                        self.y,
                        self.yaw,
                        self.path,
                        self.i)
                if(abs(self.x - self.path[-1][0]) < self.target_error and abs(self.y - self.path[-1][1]) < self.target_error):
                    v = 0.0
                    w = 0.0
                    # Mark that we just reached a goal; next selection should pick second-best
                    self.just_reached_goal = True
                    self.target = True
                    self.get_logger().info("Target reached.")
                    self.t.join()
                twist.linear.x = v
                twist.angular.z = w
                self.publisher.publish(twist)
                time.sleep(0.1)

    def target_callback(self):
        """
        Callback function to set the target flag to True.
        """
        self.exploration(
            self.map_occupancy_data,
            self.width,
            self.height,
            self.resolution,
            self.c,
            self.r,
            self.originX,
            self.originY)
        
    def scan_callback(self, msg: LaserScan):
        """
        Callback function to handle incoming LaserScan messages.

        Args:
            msg (LaserScan): The incoming LaserScan message.
        """
        self.scan_data = msg
        self.scan = msg.ranges

    def map_callback(self, msg: OccupancyGrid):
        """
        Callback function to handle incoming OccupancyGrid messages.

        Args:
            msg (OccupancyGrid): The incoming OccupancyGrid message.
        """
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.map_occupancy_data = self.map_data.data
        
        self.latest_map = msg

    def odom_callback(self, msg: Odometry):
        """
        Callback function to handle incoming Odometry messages.

        Args:
            msg (Odometry): The incoming Odometry message.
        """
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        
        self.latest_odom = msg
        
    
    def euler_from_quaternion(self, x: float, y: float, z: float, w: float) -> float:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)

        Args:
            x (float): x euler
            y (float): y euler
            z (float): z euler
            w (float): w euler

        Returns:
            float: yaw
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return yaw_z

    def heuristic(self, a: tuple[int], b: tuple[int]) -> float:
        """
        Heuristic function for A* algorithm (Euclidean distance).

        Args:
            a (tuple[int]): _description_
            b (tuple[int]): _description_

        Returns:
            _type_: _description_
        """
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    # -------------------- PATH PLANNING (A*) --------------------
    def astar(self, array: list[float], start: tuple[int], goal: tuple[int]) -> list[tuple[int]] | bool:
        """
        A* pathfinding algorithm.

        Args:
            array (list[float]): _description_
            start (tuple[int]): _description_
            goal (tuple[int]): _description_

        Returns:
            list[tuple[int]] | bool: path as list of tuples or False if no path found.
        """
        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.heuristic(start, goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[start], start))

        while oheap:
            current = heapq.heappop(oheap)[1]
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data = data + [start]
                data = data[::-1]
                return data
            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                if 0 <= neighbor[0] < array.shape[0]:
                    if 0 <= neighbor[1] < array.shape[1]:                
                        if array[neighbor[0]][neighbor[1]] == 1:
                            continue
                    else:
                        continue
                else:
                    continue
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
                    
        # If no path to goal was found, return closest path to goal
        if goal not in came_from:
            closest_node = None
            closest_dist = float('inf')
            for node in close_set:
                dist = self.heuristic(node, goal)
                if dist < closest_dist:
                    closest_node = node
                    closest_dist = dist
            if closest_node is not None:
                data = []
                while closest_node in came_from:
                    data.append(closest_node)
                    closest_node = came_from[closest_node]
                data = data + [start]
                data = data[::-1]
                return data

        return False

    # -------------------- B-SPLINE --------------------
    def bspline_planning(self, array: list[tuple], sn: int) -> list[tuple]:
        """
        B-spline path smoothing.

        Args:
            array (list[tuple]): _description_
            sn (int): _description_

        Returns:
            list[tuple]: Smoothed path as list of tuples.
        """
        try:
            array = np.array(array)
            x = array[:, 0]
            y = array[:, 1]
            N = 2
            t = range(len(x))
            x_tup = si.splrep(t, x, k=N)
            y_tup = si.splrep(t, y, k=N)
            
            x_list = list(x_tup)
            xl = x.tolist()
            x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]
            
            y_list = list(y_tup)
            yl = y.tolist()
            y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]
            
            ipl_t = np.linspace(0.0, len(x) - 1, sn)
            rx = si.splev(ipl_t, x_list)
            ry = si.splev(ipl_t, y_list)
            path = [(rx[i],ry[i]) for i in range(len(rx))]
        except:
            path = array
        return path

    # -------------------- PURE PURSUIT --------------------
    def pure_pursuit(self, 
        current_x: float,
        current_y: float,
        current_heading: float,
        path: list[tuple[float,float]],
        index: int) -> tuple[float,float,int]:
        """
        Pure pursuit path tracking algorithm.

        Args:
            current_x (float): _description_
            current_y (float): _description_
            current_heading (float): _description_
            path (list[tuple[float,float]]): _description_
            index (int): _description_

        Returns:
            tuple[float,float,int]: velocity, steering angle, updated index.
        """
        closest_point = None
        v = self.speed
        for i in range(index,len(path)):
            x = path[i][0]
            y = path[i][1]
            distance = math.hypot(current_x - x, current_y - y)
            if self.lookahead_distance < distance:
                closest_point = (x, y)
                index = i
                break
        if closest_point is not None:
            target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
            desired_steering_angle = target_heading - current_heading
        else:
            target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
            desired_steering_angle = target_heading - current_heading
            index = len(path)-1
        if desired_steering_angle > math.pi:
            desired_steering_angle -= 2 * math.pi
        elif desired_steering_angle < -math.pi:
            desired_steering_angle += 2 * math.pi
        if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
            sign = 1 if desired_steering_angle > 0 else -1
            desired_steering_angle = sign * math.pi/4
            v = 0.0

        return v,desired_steering_angle,index

    # -------------------- FRONTIER DETECTION --------------------
    def frontierB(self, matrix: list[float]) -> list[float]:
        """
        Detect frontiers in the occupancy grid.

        Args:
            matrix (list[float]): Occupancy grid matrix.

        Returns:
            list[float]: Matrix with frontiers marked.
        """
        for i in range(len(matrix)):
            for j in range(len(matrix[i])):
                if matrix[i][j] == 0.0:
                    has_unknown = False
                    has_occupied = False
                    for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                        ni, nj = i+dx, j+dy
                        if 0 <= ni < len(matrix) and 0 <= nj < len(matrix[0]):
                            if matrix[ni][nj] < 0:
                                has_unknown = True
                            if matrix[ni][nj] == VAL_OCCUPIED_MATRIX:
                                has_occupied = True
                    if has_unknown and has_occupied:
                        matrix[i][j] = 2
        return matrix

    # -------------------- GROUPING --------------------
    def directions_generator(self, _range: int = 1) -> list[tuple[int, int]]:
        """
        Generate neighbor directions for a square grid.

        Args:
            _range (int): neighborhood radius (>=1).
                        Example: 1 → 8 neighbors, 2 → 24 neighbors, etc.

        Returns:
            list[tuple[int, int]]: list of (dx, dy) offsets.
        """
        directions = []
        for dx in range(-_range, _range + 1):
            for dy in range(-_range, _range + 1):
                if dx == 0 and dy == 0:
                    continue
                directions.append((dx, dy))
        return directions
    
    def assign_groups(self,
            matrix: list[list[float]],
            value_marker: int,
            visited_marker: int,
            directions: list[tuple[int, int]] = directions_generator(1)
        ) -> tuple[list[float], dict[int, list[tuple[int, int]]]]:
        """
        Assign groups to connected components in the matrix using DFS.

        Args:
            matrix (list[list[float]]): _description_
            value_marker (int): _description_
            visited_marker (int): _description_
            directions (list[tuple[int, int]], optional): _description_. Defaults to directions_generator(1).

        Returns:
            tuple[list[float], dict[int, list[tuple[int, int]]]]: _description_
        """
        group = 1
        groups = {}
        for i in range(len(matrix)):
            for j in range(len(matrix[0])):
                if matrix[i][j] == value_marker:
                    group = self.dfs(matrix, i, j, group, groups, value_marker, visited_marker, directions)

        return matrix, groups
    
    def dfs(self, 
            matrix: list[list[float]],
            i: int,
            j: int,
            group: int,
            groups: dict[int, list[tuple[int, int]]],
            value_marker: int,
            visited_marker: int,
            directions: list[tuple[int, int]] # Directions (8-connected grid)
        ) -> int:
        """
        Depth-First Search to assign group IDs to connected components.

        Args:
            matrix (list[list[float]]): _description_
            i (int): _description_
            j (int): _description_
            group (int): _description_
            groups (dict[int, list[tuple[int, int]]]): _description_
            value_marker (int): _description_
            visited_marker (int): _description_
            directions (list[tuple[int, int]], optional): _description_. Defaults to [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,-1),(-1,1),(1,-1)]#Directions(8-connected grid).

        Returns:
            int: Next group ID.
            
        Comments:
            directions can be modified to change connectivity (4-connected, 8-connected, etc.) or increase search radius due to some usage cases like noise in map or missing connections in diagonal directions
        """
        stack = [(i, j)]

        while stack:
            x, y = stack.pop()

            # Skip invalid positions
            if x < 0 or x >= len(matrix) or y < 0 or y >= len(matrix[0]):
                continue
            if matrix[x][y] != value_marker:
                continue

            # Assign to group
            if group in groups:
                groups[group].append((x, y))
            else:
                groups[group] = [(x, y)]

            # Mark visited
            matrix[x][y] = visited_marker

            # Push all neighbors
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                stack.append((nx, ny))

        return group + 1

    # -------------------- FRONTIER SELECTION --------------------
    def fGroups(self, groups: dict[int, list[tuple[int, int]]]) -> list[tuple[int, list[tuple[int, int]]]]:
        """
        Rank frontier groups by size (largest first).

        Args:
            groups (dict[int, list[tuple[int, int]]]): _description_

        Returns:
            list[tuple[int, list[tuple[int, int]]]]: Sorted list of groups by size.
        """
        sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
        return sorted_groups

    def calculate_centroid(self, x_coords: list[int], y_coords: list[int]) -> tuple[int, int]:
        """
        Calculate the centroid of a set of points.

        Args:
            x_coords (list[int]): x coordinates of points.
            y_coords (list[int]): y coordinates of points.

        Returns:
            tuple[int, int]: Centroid coordinates as (x, y).
        """
        n = len(x_coords)
        return (int(sum(x_coords) / n), int(sum(y_coords) / n))

    def frontier_touches_boundary(
        self,
        frontier_group: list[tuple[int, int]],
        obstacle_group: list[tuple[int, int]]
    ) -> bool:
        """
        Check if any cell in the frontier group is within a threshold distance

        Args:
            frontier_group (list[tuple[int, int]]): _description_
            obstacle_group (list[tuple[int, int]]): _description_

        Returns:
            bool: True if any cell in frontier_group is within threshold distance of any cell in obstacle_group, False otherwise.
        """
        if not obstacle_group:
            return False

        threshold = 3 * self.robot_r / self.resolution  # in grid cells
        min_dist = float('inf')

        for fx, fy in frontier_group:
            for ox, oy in obstacle_group:
                dist = math.hypot(fx - ox, fy - oy)
                if dist < min_dist:
                    min_dist = dist
                    if min_dist <= threshold:
                        return True  # early exit

        # If after scanning all cells min_dist is still greater → too far
        return False

    def findClosestGroup(self, 
        map_occupancy_data: list[int],
        matrix: list[float],
        groups: list[tuple[int, list[tuple[int, int]]]],
        current: tuple[int],
        resolution: float,
        originX: float,
        originY: float) -> list[tuple[float,float]] | None:
        """
        Select the best frontier group and plan path to its centroid.

        Args:
            map_occupancy_data (list[int]): _description_
            matrix (list[float]): _description_
            groups (list[tuple[int, list[tuple[int, int]]]]): _description_
            current (tuple[int]): _description_
            resolution (float): _description_
            originX (float): _description_
            originY (float): _description_

        Returns:
            list[tuple[float,float]] | None: Path to the chosen group's centroid or None if no path found.
        """
        
        targetP = None

        _, obstacle_groups = self.assign_groups(map_occupancy_data, VAL_OCCUPIED, -128, self.directions_generator(3))
        largest_obstacle = max(obstacle_groups.items(), key=lambda x: len(x[1]))[1] if obstacle_groups else []
        self.largest_obstacle = largest_obstacle
        
        # Build ranked candidate list (frontiers that touch the largest obstacle), by ascending distance
        candidates = []
        for gid, frontier_group in groups:
            if self.frontier_touches_boundary(frontier_group, largest_obstacle):
                centroid = self.calculate_centroid([p[0] for p in frontier_group], [p[1] for p in frontier_group])
                dist = self.heuristic(current, centroid)
                candidates.append((dist, centroid, frontier_group))
        candidates.sort(key=lambda x: x[0])

        chosen_group = None
        chosen_centroid = None

        if candidates:
            # Default to best (closest) candidate
            idx = 0
            # If we just reached a goal frontier, pick second-best when available
            if self.just_reached_goal and len(candidates) >= 2:
                idx = 1
            chosen_group = candidates[idx][2]
            chosen_centroid = candidates[idx][1]
            # Reset the flag after using it
            self.just_reached_goal = False
            self.get_logger().info(f"Chosen group within {3*self.robot_r/self.resolution:.2f} cells of largest obstacle; {len(chosen_group) if chosen_group else 0} cells (rank idx={idx})")
        else:
            # Fallback: choose by size (already sorted in fGroups), allow second-best if flag is set
            if groups:
                idx = 0
                if self.just_reached_goal and len(groups) >= 2:
                    idx = 1
                chosen_group = groups[idx][1]
                # Reset the flag after using it
                self.just_reached_goal = False
                self.get_logger().info(f"No group close enough to the largest obstacle, choosing {'2nd' if idx==1 else 'largest'} group")

        if chosen_group:
            middle = chosen_centroid if chosen_centroid is not None else self.calculate_centroid([p[0] for p in chosen_group], [p[1] for p in chosen_group])
            path = self.astar(matrix, current, middle)
            path = [(p[1]*self.resolution+originX,p[0]*self.resolution+originY) for p in path]
            targetP = path

        return targetP

    # -------------------- COSTMAP + EXPLORATION --------------------
    def pathLength(self, path: list[tuple[float,float]]) -> float:
        """
        Calculate the length of a path.

        Args:
            path (list[tuple[float,float]]): Path as list of (x, y) tuples.

        Returns:
            float: Path length.
        """
        points = np.array(path)
        differences = np.diff(points, axis=0)
        distances = np.hypot(differences[:,0], differences[:,1])
        return np.sum(distances)

    def costmap(self, 
        map_occupancy_data: list[int],
        width: int,
        height: int,
        resolution: float) -> list[float]:
        """
        Create a costmap by expanding obstacles.

        Args:
            map_occupancy_data (list[int]): _description_
            width (int): _description_
            height (int): _description_
            resolution (float): _description_

        Returns:
            list[float]: Costmap matrix.
        """
        
        map_occupancy_data = np.array(map_occupancy_data).reshape(height,width)
        wall = np.where(map_occupancy_data == VAL_OCCUPIED)
        for i in range(-self.expansion_size,self.expansion_size+1):
            for j in range(-self.expansion_size,self.expansion_size+1):
                if i  == 0 and j == 0:
                    continue
                x = wall[0]+i
                y = wall[1]+j
                x = np.clip(x,0,height-1)
                y = np.clip(y,0,width-1)
                map_occupancy_data[x,y] = VAL_OCCUPIED
        map_occupancy_data = map_occupancy_data*self.resolution

        return map_occupancy_data

    def exploration(self, 
        map_occupancy_data: list[int] | list[float],
        width: int,
        height: int,
        resolution: float,
        column: int,
        row: int,
        originX: float,
        originY: float):
        """
        Main exploration function.

        Args:
            map_occupancy_data (list[int] | list[float]): _description_
            width (int): _description_
            height (int): _description_
            resolution (float): _description_
            column (int): _description_
            row (int): _description_
            originX (float): _description_
            originY (float): _description_
        """

        global pathGlobal
        
        raw_grid = np.array(map_occupancy_data).reshape(height, width)
        self.get_logger().debug(f"is_fully_enclosed: checking at robot grid {(row, column)}")
        if self.is_fully_enclosed(raw_grid, (row, column)):
            self.get_logger().info("The area is fully enclosed → exploration finished")
            pathGlobal = -1
            return

        matrix = self.costmap(
            map_occupancy_data,
            width,
            height,
            self.resolution)
        matrix[row][column] = 0
        matrix[matrix >= VAL_OCCUPIED*self.resolution] = VAL_OCCUPIED_MATRIX
        matrix = self.frontierB(matrix)
        matrix, groups = self.assign_groups(matrix, VAL_FRONTIER, 0)
        
        groups = self.fGroups(groups)
        self.last_frontier_groups = groups
        
        map_occupancy_data = np.array(map_occupancy_data).reshape(height,width)
        
        if len(groups) == 0:
            path = -1
        else:
            matrix[matrix < 0] = 1
            path = self.findClosestGroup(
                map_occupancy_data,
                matrix,
                groups,
                (row,column),
                self.resolution,
                originX,
                originY)
            if path != None:
                path = self.bspline_planning(path, len(path)*5)
            else:
                path = -1
        pathGlobal = path
        
        return

    # -------------------- LOCAL CONTROL --------------------
    def localControl(self, scan: list[tuple[float, float]]) -> tuple[float,float | None,None]:
        """
        Simple reactive obstacle avoidance based on laser scan data.

        Args:
            scan (list[tuple[float, float]]): _description_

        Returns:
            tuple[float,float | None,None]: velocity, angular velocity
        """
        v = None
        w = None
        for i in range(60):
            if scan[i] < self.robot_r:
                v = 0.2
                w = -math.pi/4 
                break
        if v == None:
            for i in range(300,360):
                if scan[i] < self.robot_r:
                    v = 0.2
                    w = math.pi/4
                    break
        return v,w
    
    # -------------------- FULLY ENCLOSURE --------------------
    def _disk_structure(self, radius_pixels: int) -> np.ndarray:
        """
        Create a disk-shaped structuring element for morphological operations.

        Args:
            radius_pixels (int): Radius of the disk in pixels.

        Returns:
            np.ndarray: 2D boolean array representing the disk.
        """
        L = 2 * radius_pixels + 1
        cy = cx = radius_pixels
        y, x = np.ogrid[:L, :L]
        mask = (x - cx) ** 2 + (y - cy) ** 2 <= radius_pixels ** 2
        return mask

    def is_fully_enclosed(self, raw_grid: np.ndarray, position: tuple[int, int]) -> bool:
        """
        Determine if the robot is fully enclosed by obstacles in the occupancy grid.

        Args:
            raw_grid (np.ndarray): 2D array representing the occupancy grid.
            position (tuple[int, int]): Robot's position in grid coordinates (row, col).

        Returns:
            bool: True if fully enclosed, False otherwise.
        """
        if raw_grid is None or raw_grid.size == 0:
            return False

        h, w = raw_grid.shape
        r0, c0 = position
        if not (0 <= r0 < h and 0 <= c0 < w):
            return False

        occupancy = (raw_grid >= VAL_OCCUPIED)

        r_pix = max(0, int(math.ceil(self.robot_r / self.resolution)))

        # Build disk structuring element and dilate obstacle mask
        if r_pix > 0:
            structure = self._disk_structure(r_pix)
            occupancy_dilated = ndimage.binary_dilation(occupancy, structure=structure)
        else:
            occupancy_dilated = occupancy.copy()

        # free_eroded: True where center of robot can be placed (i.e. not inside inflated obstacles)
        free_eroded = ~occupancy_dilated

        # If robot is inside the inflated obstacle band, try to find a nearby start cell within a small neighborhood (search radius = max(1, r_pix)). If not found, avoid declaring enclosed (conservative).
        start_r, start_c = r0, c0
        if not free_eroded[start_r, start_c]:
            search_px = max(1, r_pix)
            found = False
            for dr in range(-search_px, search_px + 1):
                for dc in range(-search_px, search_px + 1):
                    nr, nc = r0 + dr, c0 + dc
                    if 0 <= nr < h and 0 <= nc < w and free_eroded[nr, nc]:
                        start_r, start_c = nr, nc
                        found = True
                        break
                if found:
                    break
            if not found:
                # prevent false positive enclosure when robot is inside inflated obstacle area with no nearby free cell
                self.get_logger().debug(
                    f"is_fully_enclosed: robot center inside inflated obstacle area; "
                    f"no nearby free cell (r_pix={r_pix}) — treating as NOT enclosed")
                return False

        # BFS over free_eroded (8-connected), if we can reach the map border -> NOT enclosed
        visited = np.zeros_like(free_eroded, dtype=bool)
        q = deque()
        q.append((start_r, start_c))
        visited[start_r, start_c] = True
        directions = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]

        while q:
            x, y = q.popleft()
            # If any reachable cell touches the map edge -> the robot can escape
            if x == 0 or y == 0 or x == h - 1 or y == w - 1:
                return False
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < h and 0 <= ny < w and not visited[nx, ny] and free_eroded[nx, ny]:
                    visited[nx, ny] = True
                    q.append((nx, ny))

        return True


    # -------------------- MAP DUMP --------------------
    def mark_frontiers_and_goal_on_dump_map(self):
        """
        Mark frontiers, next goal, and largest obstacle on the latest map data for visualization and saving.
        """
        if self.map_dump and self.latest_map:
            dumped_map = np.array(self.latest_map.data).reshape(self.height, self.width)

            # Mark frontiers
            if hasattr(self, 'last_frontier_groups'):
                for gid, frontier_group in self.last_frontier_groups:
                    for (fx, fy) in frontier_group:
                        dumped_map[fx][fy] = VAL_FRONTIER

            # Mark next goal
            if hasattr(self, 'c') and hasattr(self, 'r'):
                dumped_map[self.r][self.c] = VAL_NEXT_GOAL
                
            # Mark largest obstacle
            if hasattr(self, 'largest_obstacle') and self.largest_obstacle:
                for (ox, oy) in self.largest_obstacle:
                    dumped_map[ox][oy] = VAL_LARGEST_OBSTACLE

            # Update map data
            self.latest_map.data = dumped_map.flatten().tolist()

            # Save map + odometry
            self.combine_map_odom_data()


    def combine_map_odom_data(self, odom: Odometry=None, mapp: OccupancyGrid=None):
        """
        Combine the latest map and odometry data and save to a JSON file.

        Args:
            odom (Odometry, optional): _description_. Defaults to None.
            mapp (OccupancyGrid, optional): _description_. Defaults to None.
        """
        if odom == None: odom = self.latest_odom
        if mapp == None: mapp = self.latest_map

        try:
            map_data = {
                'header': {
                    'stamp': {
                        'sec': mapp.header.stamp.sec,
                        'nanosec': mapp.header.stamp.nanosec
                    },
                    'frame_id': mapp.header.frame_id
                },
                'info': {
                    'map_load_time': {
                        'sec': mapp.info.map_load_time.sec,
                        'nanosec': mapp.info.map_load_time.nanosec
                    },
                    'resolution': mapp.info.resolution,
                    'width': mapp.info.width,
                    'height': mapp.info.height,
                    'origin': {
                        'position': {
                            'x': mapp.info.origin.position.x,
                            'y': mapp.info.origin.position.y,
                            'z': mapp.info.origin.position.z
                        },
                        'orientation': {
                            'x': mapp.info.origin.orientation.x,
                            'y': mapp.info.origin.orientation.y,
                            'z': mapp.info.origin.orientation.z,
                            'w': mapp.info.origin.orientation.w
                        }
                    }
                },
                'data': list(mapp.data)
            }

            odom_data = {
                'header': {
                    'stamp': {
                        'sec': odom.header.stamp.sec,
                        'nanosec': odom.header.stamp.nanosec
                    },
                    'frame_id': odom.header.frame_id
                },
                'pose': {
                    'pose': {
                        'position': {
                            'x': odom.pose.pose.position.x,
                            'y': odom.pose.pose.position.y,
                            'z': odom.pose.pose.position.z
                        },
                        'orientation': {
                            'x': odom.pose.pose.orientation.x,
                            'y': odom.pose.pose.orientation.y,
                            'z': odom.pose.pose.orientation.z,
                            'w': odom.pose.pose.orientation.w
                        }
                    }
                },
            }

            res_data = {
                'odom': odom_data,
                'map': map_data
            }

            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filename = os.path.join(self.output_directory, f'map_odom_{timestamp}.json')

            with open(filename, 'w') as json_file:
                json.dump(res_data, json_file, indent=4)

            self.get_logger().info(f"Saved map data to {filename}")

        except Exception as e:
            self.get_logger().warning(f"Failed to process and save map and odom data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryExploration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
