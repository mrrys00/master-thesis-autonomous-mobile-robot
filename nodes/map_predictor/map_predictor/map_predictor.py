import os 
import json
from datetime import datetime
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapProcessorNode(Node):
    def __init__(self):
        super().__init__('map_processor_node')

        # Declare parameters
        self.declare_parameter('output_directory', 'messages')
        self.declare_parameter('save_json', False)
        self.declare_parameter('fixed_fig_size', -1)

        # Get parameter values
        self.output_directory = self.get_parameter('output_directory').get_parameter_value().string_value
        self.save_json = self.get_parameter('save_json').get_parameter_value().bool_value
        self.fixed_fig_size = self.get_parameter('fixed_fig_size').get_parameter_value().integer_value
        
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

        self.get_logger().info(f"Subscribed to /map topic. Messages will be saved in {self.output_directory} if save_json is True.")

    def _save_map(
            self,
            msg: OccupancyGrid,
            num_estimated_figures: int,
            greatest_circumference: int,
            total_circumference: int,
            accuracy: float):
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
            'data': list(msg.data),
            'num_estimated_figures': int(num_estimated_figures),
            'greatest_circumference': int(greatest_circumference),
            'total_circumference': int(total_circumference),
            'accuracy': float(accuracy),
        }

        # Generate a timestamped filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        filename = os.path.join(self.output_directory, f'map_{timestamp}.json')

        # Write data to a JSON file
        with open(filename, 'w') as json_file:
            json.dump(map_data, json_file, indent=4)

        self.get_logger().info(f"Saved map data to {filename}")

    def _calculate_metricies(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        data = list(msg.data)
        grid = np.array(data).reshape((height, width))
        walls = np.column_stack(np.where(grid == 100))

        # Clustering DBSCAN
        clustering = DBSCAN(eps=5, min_samples=5).fit(walls)
        clusters = clustering.labels_

        num_estimated_figures = 0
        greatest_circumference = self.fixed_fig_size
        total_circumference = 0

        for cluster_id in set(clusters):
            if cluster_id == -1:  # Omit noise
                continue

            cluster_points = walls[clusters == cluster_id]

            # Check if the cluster is large enough
            if len(cluster_points) < 5:  # Filter small clusters (less than 5 points)
                continue

            # Calculate bounding rectangle
            min_y, min_x = np.min(cluster_points, axis=0)
            max_y, max_x = np.max(cluster_points, axis=0)

            # Calculate circumference
            width = max_x - min_x + 1
            height = max_y - min_y + 1
            circumference = 2 * (width + height)

            # Update statistics
            num_estimated_figures += 1
            total_circumference += circumference
            if self.fixed_fig_size != -1: greatest_circumference = max(greatest_circumference, circumference)

        # Log results
        self.get_logger().info(f"Number of estimated figures: {num_estimated_figures}")
        self.get_logger().info(f"Circumference of the greatest figure: {greatest_circumference}")
        self.get_logger().info(f"Total circumference of all estimated figures: {total_circumference}")
        accuracy = round(total_circumference / (num_estimated_figures * greatest_circumference), 2)
        self.get_logger().info(f"Accuracy: {accuracy * 100}%")

        return grid, num_estimated_figures, greatest_circumference, total_circumference, accuracy

    def map_callback(self, msg: OccupancyGrid):
        try:
            _, num_estimated_figures, greatest_circumference, total_circumference, accuracy = self._calculate_metricies(msg)
            if self.save_json:  # Use the save_json parameter to determine if the map should be saved
                self._save_map(msg, num_estimated_figures, greatest_circumference, total_circumference, accuracy)

        except Exception as e:
            self.get_logger().error(f"Failed to process and save map data: {e}")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MapProcessorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
