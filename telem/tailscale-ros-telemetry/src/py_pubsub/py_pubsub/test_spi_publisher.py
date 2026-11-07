#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(String, 'spi_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # every 100ms

    def timer_callback(self):
        snap = {'sensor1': random.randint(0, 1023), 'sensor2': random.randint(0, 1023)}
        msg = String()
        msg.data = json.dumps(snap)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {snap}")

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
