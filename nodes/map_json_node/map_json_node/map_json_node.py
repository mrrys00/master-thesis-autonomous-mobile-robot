import os
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

class MapToJsonNode(Node):
    def __init__(self):
        super().__init__('map_to_json_node')

        # Declare parameters
        self.declare_parameter('output_directory', 'messages')

        # Get output directory
        self.output_directory = self.get_parameter('output_directory').get_parameter_value().string_value
        
        # Ensure output directory exists
        if not os.path.exists(self.output_directory):
            os.makedirs(self.output_directory)

        # Subscribe to the /map topic
        self.map_subscription = self.create_subscription(
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

        self.get_logger().info(f"Subscribed to /map topic. Messages will be saved in {self.output_directory}.")

        self.latest_map: OccupancyGrid | dict = None
        self.latest_odom: Odometry | dict = None

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

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

        self.combine_map_odom_data()

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

def main(args=None):
    rclpy.init(args=args)

    try:
        node = MapToJsonNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
