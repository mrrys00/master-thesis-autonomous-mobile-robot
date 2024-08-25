import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Change this to the appropriate message type

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor_node')

        # Publisher on topic2
        self.publisher_ = self.create_publisher(String, 'topic2', 10)

        # Subscriber on topic1
        self.subscription = self.create_subscription(
            String,
            'topic1',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # Processing the data
        processed_data = self.process_data(msg.data)

        # Publishing the processed data
        self.publisher_.publish(String(data=processed_data))
        self.get_logger().info(f"Received {msg.data}, Published {processed_data}")

    def process_data(self, data):
        # An example data processing function.
        # Modify this according to your needs.
        return data.upper()  # Converts the string to uppercase

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

