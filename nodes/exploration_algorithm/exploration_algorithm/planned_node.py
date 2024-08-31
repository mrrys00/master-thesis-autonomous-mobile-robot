import rclpy

from geometry_msgs.msg import PoseStamped, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage

from math import pi, sin, cos
from random import choice as rand_choice
from collections import deque

FIND_GOAL_DELAY = 5.0
GOAL_DELTA = 0.05
MAX_GOAL_RETRIES = 5

class ExplorationAlgorithm(Node):

    def __init__(self):
        super().__init__('exploration_algorithm')
        self.get_logger().info('Exploration algorithm is running!')

        # Subscribe /map
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        # Subscribe /tf
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
        
        # Subscribe /scan
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.amcl_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_pose_callback,
            10
        )
        
        # Create navigation client
        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        while not self.navigation_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Waiting for navigation server...')

        self.last_goal_set_stamp: Time = self.get_clock().now()

        def _init_position() -> TransformStamped:
            transform = TransformStamped()
            transform.header.frame_id = 'odom'
            transform.header.stamp = self.get_clock().now().to_msg()

            transform.child_frame_id = 'base_link'
            
            # actual position
            transform.transform.translation.x = 0.0     # actual x
            transform.transform.translation.y = 0.0     # actual y
            transform.transform.translation.z = 0.0     # always 0

            # actual rotation
            transform.transform.rotation.x = 0.0        # always 0
            transform.transform.rotation.y = 0.0        # always 0
            transform.transform.rotation.z = 1.0        # actual for pi/2 it's sin(pi/2) = 1.0
            transform.transform.rotation.w = 0.0        # always for pi/2 it's cos(pi/2) = 0.0

            return transform

        self.latest_position: TransformStamped = _init_position()
        self.latest_scan: LaserScan = LaserScan()
        self.latest_map: OccupancyGrid = OccupancyGrid()
        self.latest_amcl_pose: PoseWithCovarianceStamped= PoseWithCovarianceStamped()
        self.latest_goal: PoseStamped = PoseStamped()
        self.goal_retries: int = 0

        self.find_goal_timer = self.create_timer(FIND_GOAL_DELAY, self.find_goal)

    def map_callback(self, msg: OccupancyGrid):
        """
        Take latest map
        """
        self.latest_map = msg

    def tf_callback(self, msg: TransformStamped):
        """
        Take tf message and change it to the latest position
        """
        msg = self.convert_tf_message(msg)
        if msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link':
            self.latest_position = msg

    def scan_callback(self, msg: LaserScan):
        """
        Take scan message
        """
        self.latest_scan = msg

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        Take amcl pose message
        """
        self.latest_amcl_pose = msg

    def verify_goal(self):
        """
        Verify if robot reached its goal
        """
        if self.goal_retries >= MAX_GOAL_RETRIES:
            self.get_logger().warning(f"Old goal x={self.latest_goal.pose.position.x} y={self.latest_goal.pose.position.y} not reached -> allow to find new goal")
            return True

        self.goal_retries += 1
        return \
            (self.latest_amcl_pose.pose.pose.position.x - self.latest_goal.pose.position.x)**2 + \
            (self.latest_amcl_pose.pose.pose.position.y - self.latest_goal.pose.position.y)**2 <= GOAL_DELTA

    def find_goal(self):
        """
        Find next goal based on nearest unexplored cell choose
        """
        if not self.verify_goal():
            self.get_logger().info(f"Robot is trying to reach its goal x={self.latest_goal.pose.position.x} y={self.latest_goal.pose.position.y}")
            return

        self.goal_retries = 0
        _resolution = self.latest_map.info.resolution
        _width, _height = self.latest_map.info.width, self.latest_map.info.height
        _robot_x, _robot_y = self.latest_position.transform.translation.x, self.latest_position.transform.translation.y
        # _robot_x, _robot_y = self.latest_amcl_pose.pose.pose.position.x, self.latest_amcl_pose.pose.pose.position.y
        _origin_x, _origin_y = self.latest_map.info.origin.position.x, self.latest_map.info.origin.position.y
        _linear_grid = list(self.latest_map.data)
        unknown_space = -1
        free_space = 0
        occupied_space = 100
        unreachable_position_marker = 100
        robot_position_marker = 2
        goal_position_marker = 5

        if _resolution == 0:
            self.get_logger().info(f"Resolution {_resolution} error")
            return

        # calculate and mark robot position in grid 
        grid_x, grid_y = int((_robot_x - _origin_x) / _resolution), int((_robot_y - _origin_y) / _resolution)
        if 0 <= grid_x < _width and 0 <= grid_y < _height:
            linear_idx = grid_y *_width + grid_x
            _linear_grid[linear_idx] = robot_position_marker
            self.get_logger().info(f"Robot position updated to {linear_idx}")

        # convert to 2d list
        _2d_list = []
        for i in range(_height):
            row = _linear_grid[i*_width : (i+1)*_width]
            _2d_list.append(row)

        _2d_list = self.process_grid_to_find_new_goal(
            _2d_list,
            unknown_space,
            free_space,
            occupied_space,
            unreachable_position_marker,
            robot_position_marker,
            goal_position_marker
        )

        _goal_x, _goal_y = None, None
        for _y, _row in enumerate(_2d_list):
            for _x, _cell in enumerate(_row):
                if _cell == goal_position_marker:
                    _goal_x, _goal_y = _origin_x + _x*_resolution, _origin_y + _y*_resolution
                    break
        else:
            self.get_logger().warning("Goal position not found!")

        new_cords = PoseStamped()
        if _goal_y is not None and _goal_x is not None:
            new_cords = self.prepare_pose_stamped(_goal_x, _goal_y)
            self.send_navigation_goal(new_cords)
            self.get_logger().info(f"Navigation goal sent to x={_goal_x} y={_goal_y}")
        else:
            self.get_logger().warning("Navigation goal NOT sent")
            
    def process_grid_to_find_new_goal(
            self, 
            _grid: list[list],
            unknown_space: int,
            free_space: int,
            occupied_space: int,
            unreachable_position_marker: int,
            robot_position_marker: int,
            goal_position_marker: int
        ) -> tuple[int, int]:
        """
        Finds the possible nearest point to explore
        """
        # Find the position of robot
        start = None
        for i in range(len(_grid)):
            for j in range(len(_grid[i])):
                if _grid[i][j] == robot_position_marker:
                    start = (i, j)
                    break
            if start:
                break
        
        self.get_logger().info(f"Start position found as {start}")
        # BFS setup
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
        queue = deque([start])
        visited = set()
        visited.add(start)
        found_unknown = False

        while queue and not found_unknown:
            x, y = queue.popleft()

            for dx, dy in directions:
                nx, ny = x + dx, y + dy

                # Ensure within bounds and not already visited
                if 0 <= nx < len(_grid) and 0 <= ny < len(_grid[0]) and (nx, ny) not in visited:
                    if _grid[nx][ny] == free_space:  # Free space, continue exploring
                        queue.append((nx, ny))
                        visited.add((nx, ny))
                    elif _grid[nx][ny] == unknown_space:  # Found unknown terrain
                        _grid[nx][ny] = goal_position_marker
                        found_unknown = True
                        break

        # Mark unreachable space
        for i in range(len(_grid)):
            for j in range(len(_grid[i])):
                if _grid[i][j] == free_space and (i, j) not in visited:
                    _grid[i][j] = unreachable_position_marker

        return _grid

    def prepare_pose_stamped(self, _x: float, _y: float) -> PoseStamped:
        """
        Prepare PoseStamped message structure from goal x and y
        """
        # _x, _y, _theta = self.process_to_cords(straight_dist, angle_delta)

        _theta = 0.5    # hardcode
        goal_pose = PoseStamped()
        goal_pose.header = Header()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = _x
        goal_pose.pose.position.y = _y
        goal_pose.pose.orientation.z = sin(_theta / 2.0)
        goal_pose.pose.orientation.w = cos(_theta / 2.0)

        return goal_pose

    # def prepare_pose_stamped(self, straight_dist: float, angle_delta: float) -> PoseStamped:
    #     """
    #     Prepare PoseStamped message structure from distance and actual position
    #     """
    #     _x, _y, _theta = self.process_to_cords(straight_dist, angle_delta)

    #     goal_pose = PoseStamped()
    #     goal_pose.header = Header()
    #     goal_pose.header.frame_id = 'map'
    #     goal_pose.header.stamp = self.get_clock().now().to_msg()
        
    #     goal_pose.pose.position.x = _x
    #     goal_pose.pose.position.y = _y
    #     goal_pose.pose.orientation.z = sin(_theta / 2.0)
    #     goal_pose.pose.orientation.w = cos(_theta / 2.0)

    #     return goal_pose

    def process_to_cords(self, straight_dist: float, angle_delta: float) -> tuple[int, int] or None:
        """
        Process stright line dist to cords based on actual position
        """
        actual_angle = euler_from_quaternion([
            self.latest_position.transform.rotation.x,
            self.latest_position.transform.rotation.y,
            self.latest_position.transform.rotation.z,
            self.latest_position.transform.rotation.w
        ])[2]
        actual_x = self.latest_position.transform.translation.x
        actual_y = self.latest_position.transform.translation.y
        
        new_angle = (actual_angle + angle_delta) % (2*pi)
        new_x = actual_x + straight_dist*cos(new_angle)
        new_y = actual_y + straight_dist*sin(new_angle)

        return new_x, new_y, new_angle 

    def send_navigation_goal(self, goal_pose):
        """
        Send navigation goal
        """
        self.get_logger().info(f'Sending navigation goal as xyzw: ({goal_pose.pose.position.x}, {goal_pose.pose.position.y}, {goal_pose.pose.orientation.z}, {goal_pose.pose.orientation.w})')

        self.latest_goal = goal_pose
        # prepare goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.navigation_client.send_goal_async(goal_msg)

    def convert_tf_message(self, tf_message: TFMessage) -> TransformStamped or None:
        """"
        Converts TFMessage to TransformStamped message
        """
        if isinstance(tf_message, TFMessage):
            transforms_list = tf_message.transforms
        else:
            self.get_logger().debug("Input is not a TFMessage")
            return None

        if transforms_list:
            transform_stamped = transforms_list[0]
            if isinstance(transform_stamped, TransformStamped):
                return transform_stamped
            else:
                self.get_logger().debug("TransformStamped not found in TFMessage")
        else:
            self.get_logger().debug("TFMessage is empty")
        
        return None

def main(args=None):
    rclpy.init(args=args)

    exploration_algorithm = ExplorationAlgorithm()

    rclpy.spin(exploration_algorithm)

    exploration_algorithm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
