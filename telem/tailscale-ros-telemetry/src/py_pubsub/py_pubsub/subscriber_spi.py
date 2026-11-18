#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class SpiSubscriber(Node):
    def __init__(self):
        super().__init__('spi_subscriber')
        self.subscription = self.create_subscription(
            String,
            'spi_data',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        try:
            data = json.loads(msg.data)  # convert JSON string back to dict
            self.get_logger().info(f"Received SPI data: {data}")
        except json.JSONDecodeError:
            self.get_logger().warn(f"Failed to decode message: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SpiSubscriber()
    print("started subscriber")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
