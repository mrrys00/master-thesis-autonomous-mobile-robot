import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import heapq, math, random, yaml
import scipy.interpolate as si
import sys, threading, time

from collections import deque
import os
import json
from datetime import datetime

with open("src/config/exploration/params.yaml", 'r') as file:
    params = yaml.load(file, Loader=yaml.FullLoader)

lookahead_distance = params["lookahead_distance"]
speed = params["speed"]
expansion_size = params["expansion_size"]
target_error = params["target_error"]
robot_r = params["robot_r"]
resolution = params["resolution"]

pathGlobal = 0

VAL_FRONTIER = 2

VAL_UNKNOWN = -1
VAL_FREE = 0
VAL_OCCUPIED = 100
VAL_INACCESSIBLE = 101

VAL_CURR_POSITION = 102
VAL_NEXT_GOAL = 103

VAL_LARGEST_OBSTACLE = 104

MAP_DUMP = True

VAL_OCCUPIED_MATRIX = 1.0


# -------------------- ROS2 NODE --------------------
class BoundaryExploration(Node):
    def __init__(self):
        super().__init__('Exploration')
        
        if MAP_DUMP:
            # Declare parameters
            self.declare_parameter('output_directory', 'messages')

            # Get output directory
            self.output_directory = self.get_parameter('output_directory').get_parameter_value().string_value
            
            # Ensure output directory exists
            if not os.path.exists(self.output_directory):
                os.makedirs(self.output_directory)
                
        self.latest_map: OccupancyGrid | dict = None
        self.latest_odom: Odometry | dict = None
        
        
        self.subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        print("[INFO] EXPLORATION MODE ACTIVE")
        
        self.target: bool = True
        threading.Thread(target=self.exp).start()
        
        self.scan_data: LaserScan | None
        self.scan: list[float] | any
        
        self.map_data: OccupancyGrid | None
        self.resolution: float | any
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
        
        # Flag to pick second-best frontier right after reaching a goal frontier
        self.just_reached_goal: bool = False
        
    def exp(self):
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
                    print("[INFO] EXPLORATION COMPLETED")
                    sys.exit()
                self.c = int((self.path[-1][0] - self.originX)/self.resolution) 
                self.r = int((self.path[-1][1] - self.originY)/self.resolution) 
                self.target = False
                self.i = 0
                print("[INFO] NEW GOAL SELECTED")
                
                self.mark_frontiers_and_goal_on_dump_map()      # MAP DUMP
                
                t = self.pathLength(self.path)/speed
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
                if(abs(self.x - self.path[-1][0]) < target_error and abs(self.y - self.path[-1][1]) < target_error):
                    v = 0.0
                    w = 0.0
                    # Mark that we just reached a goal; next selection should pick second-best
                    self.just_reached_goal = True
                    self.target = True
                    print("[INFO] GOAL REACHED")
                    self.t.join()
                twist.linear.x = v
                twist.angular.z = w
                self.publisher.publish(twist)
                time.sleep(0.1)

    def target_callback(self):
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
        self.scan_data = msg
        self.scan = msg.ranges

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.map_occupancy_data = self.map_data.data
        
        self.latest_map = msg

    def odom_callback(self, msg: Odometry):
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

    def heuristic(self, a: tuple[int], b: tuple[int]):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    # -------------------- PATH PLANNING (A*) --------------------
    def astar(self, array: list[float], start: tuple[int], goal: tuple[int]) -> list[tuple[int]] | bool:
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
        # TO COMMENT If no path to goal was found, return closest path to goal
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
        global lookahead_distance
        closest_point = None
        v = speed
        for i in range(index,len(path)):
            x = path[i][0]
            y = path[i][1]
            distance = math.hypot(current_x - x, current_y - y)
            if lookahead_distance < distance:
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
            rande (int): neighborhood radius (>=1).
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
            directions: list[tuple[int, int]] = directions_generator(1)) -> tuple[list[float], dict[int, list[tuple[int, int]]]]:
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
        """_summary_

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
            int: _description_
            
        Comments:
            directions can be modified to change connectivity (4-connected, 8-connected, etc.) or increase search radius due to some usage cases like noise in map or missing connections in diagonal directions
        """

        # Use stack for iterative DFS
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
        sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
        return sorted_groups

    def calculate_centroid(self, x_coords: list[int], y_coords: list[int]) -> tuple[int, int]:
        n = len(x_coords)
        return (int(sum(x_coords) / n), int(sum(y_coords) / n))

    def frontier_touches_boundary(
        self,
        frontier_group: list[tuple[int, int]],
        obstacle_group: list[tuple[int, int]]
    ) -> bool:
        if not obstacle_group:
            return False

        threshold = 3 * robot_r / resolution  # in grid cells
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
        
        targetP = None

        _, obstacle_groups = self.assign_groups(map_occupancy_data, VAL_OCCUPIED, -128, self.directions_generator(3))  # -999 to mark visited
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
            print(f"[INFO] CHOSEN GROUP IS WITHIN {3*robot_r/resolution:.2f} CELLS OF LARGEST OBSTACLE; {len(chosen_group) if chosen_group else 0} CELLS (rank idx={idx})")
        else:
            # Fallback: choose by size (already sorted in fGroups), allow second-best if flag is set
            if groups:
                idx = 0
                if self.just_reached_goal and len(groups) >= 2:
                    idx = 1
                chosen_group = groups[idx][1]
                # Reset the flag after using it
                self.just_reached_goal = False
                print(f"[INFO] NO GROUP TOUCHES OBSTACLE, CHOOSING {'SECOND' if idx==1 else 'LARGEST'} GROUP")

        if chosen_group:
            middle = chosen_centroid if chosen_centroid is not None else self.calculate_centroid([p[0] for p in chosen_group], [p[1] for p in chosen_group])
            path = self.astar(matrix, current, middle)
            # if not path:  # If uncommented finishes the exploration too early
            #     print("[INFO] NO PATH FOUND")
            #     return None
            path = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path]
            targetP = path

        return targetP

    # -------------------- COSTMAP + EXPLORATION --------------------
    def pathLength(self, path):
        points = np.array(path)
        differences = np.diff(points, axis=0)
        distances = np.hypot(differences[:,0], differences[:,1])
        return np.sum(distances)

    def costmap(self, 
        map_occupancy_data: list[int],
        width: int,
        height: int,
        resolution: float) -> list[float]:
        
        map_occupancy_data = np.array(map_occupancy_data).reshape(height,width)
        wall = np.where(map_occupancy_data == VAL_OCCUPIED)
        for i in range(-expansion_size,expansion_size+1):
            for j in range(-expansion_size,expansion_size+1):
                if i  == 0 and j == 0:
                    continue
                x = wall[0]+i
                y = wall[1]+j
                x = np.clip(x,0,height-1)
                y = np.clip(y,0,width-1)
                map_occupancy_data[x,y] = VAL_OCCUPIED
        map_occupancy_data = map_occupancy_data*resolution

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

        global pathGlobal
        matrix = self.costmap(
            map_occupancy_data,
            width,
            height,
            resolution)
        matrix[row][column] = 0
        matrix[matrix > 5] = VAL_OCCUPIED_MATRIX
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
                resolution,
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
        v = None
        w = None
        for i in range(60):
            if scan[i] < robot_r:
                v = 0.2
                w = -math.pi/4 
                break
        if v == None:
            for i in range(300,360):
                if scan[i] < robot_r:
                    v = 0.2
                    w = math.pi/4
                    break
        return v,w
    
    # -------------------- FULLY ENCLOSURE --------------------
    def is_fully_enclosed(
        self,
        grid: np.ndarray,
        position: tuple[int, int],
        free_val: int = VAL_FREE,
        occupied_val: int = VAL_OCCUPIED,
        unknown_val: int = VAL_UNKNOWN
    ) -> bool:
        # TODO implement
        return False

    # -------------------- MAP DUMP --------------------
    def mark_frontiers_and_goal_on_dump_map(self):  # Mark frontiers and next goal in latest map copy
        if MAP_DUMP and self.latest_map:
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
        if odom == None: odom = self.latest_odom
        if mapp == None: mapp = self.latest_map

        try:
            # Prepare the message data for JSON serialization
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

            # Prepare the message data for JSON serialization
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

            # Generate a timestamped filename
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filename = os.path.join(self.output_directory, f'map_odom_{timestamp}.json')

            # Write data to a JSON file
            with open(filename, 'w') as json_file:
                json.dump(res_data, json_file, indent=4)

            self.get_logger().info(f"Saved map data to {filename}")

        except Exception as e:
            self.get_logger().error(f"Failed to process and save map and odom data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryExploration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
