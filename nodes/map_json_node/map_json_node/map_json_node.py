import os
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

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
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.get_logger().info(f"Subscribed to /map topic. Messages will be saved in {self.output_directory}.")

    def map_callback(self, msg: OccupancyGrid):
        try:
            # Prepare the message data for JSON serialization
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

            # Generate a timestamped filename
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filename = os.path.join(self.output_directory, f'map_{timestamp}.json')

            # Write data to a JSON file
            with open(filename, 'w') as json_file:
                json.dump(map_data, json_file, indent=4)

            self.get_logger().info(f"Saved map data to {filename}")

        except Exception as e:
            self.get_logger().error(f"Failed to process and save map data: {e}")

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
