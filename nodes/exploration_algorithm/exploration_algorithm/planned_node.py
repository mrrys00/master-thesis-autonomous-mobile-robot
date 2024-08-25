import rclpy

from geometry_msgs.msg import PoseStamped, TransformStamped
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

FIND_GOAL_DELAY = 16.0
GOAL_AREA_NUM = 9
ALLOWED_AVG_DIST = 0.3

class ExplorationAlgorithm(Node):

    def __init__(self):
        super().__init__('exploration_algorithm')
        self.get_logger().info('Exploration algorithm is running')

        # Subscribe /map
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
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

    def find_goal(self):
        """
        Find next goal based on Monte Carlo area choose
        """
        _areas_mean_dist: list[float] = []
        allowed_areas_idxes: list[int] = []
        latest_scans_num = len(self.latest_scan.ranges)

        full_scan_area = 3/2*pi     # 270 deg - lidar area
        single_area_radian = full_scan_area / GOAL_AREA_NUM

        for area_num in range(GOAL_AREA_NUM):
            area_dist = 0.0
            for _scan in self.latest_scan.ranges[
                latest_scans_num//GOAL_AREA_NUM*area_num
                :latest_scans_num//GOAL_AREA_NUM*(area_num+1)]:
                if _scan: area_dist += float(_scan)
            
            area_dist /= latest_scans_num/GOAL_AREA_NUM     # get mean distance for the area
            _areas_mean_dist.append(area_dist)

        # find allowed areas; area is allowed when mean scan distance is greater than ALLOWED_AVG_DIST
        for area_idx, area_mean_dist in enumerate(_areas_mean_dist):
            if area_mean_dist > ALLOWED_AVG_DIST: allowed_areas_idxes.append(area_idx)

        new_cords = PoseStamped()
        if not allowed_areas_idxes:
            # if no area allowed
            new_cords = self.prepare_pose_stamped(-0.2, pi)
        else: 
            # possible to set next goal, choose one of allowed cords
            choosen_area_idx = rand_choice(allowed_areas_idxes)
            new_cords = self.prepare_pose_stamped(
                _areas_mean_dist[choosen_area_idx] * 0.7,
                -full_scan_area/2 + choosen_area_idx*single_area_radian + single_area_radian/2
            )
        
        self.send_navigation_goal(new_cords)

    def prepare_pose_stamped(self, straight_dist: float, angle_delta: float) -> PoseStamped:
        """
        Prepare PoseStamped message structure from distance and actual position
        """
        _x, _y, _theta = self.process_to_cords(straight_dist, angle_delta)

        goal_pose = PoseStamped()
        goal_pose.header = Header()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = _x
        goal_pose.pose.position.y = _y
        goal_pose.pose.orientation.z = sin(_theta / 2.0)
        goal_pose.pose.orientation.w = cos(_theta / 2.0)

        return goal_pose

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
