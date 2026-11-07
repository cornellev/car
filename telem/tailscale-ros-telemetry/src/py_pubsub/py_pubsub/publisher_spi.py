#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .uc26_sensor_reader.read_shm import SensorShmReader
import json

class SpiPublisher(Node):
    def __init__(self):
        super().__init__('spi_publisher')
        self.publisher_ = self.create_publisher(String, 'spi_data', 10)
        #create shared memory buffer
        self.reader = SensorShmReader()
        self.timer = self.create_timer(0.02, self.timer_callback)  # ~50 Hz

    def timer_callback(self):
        # retry attaching to shared memory if reader wasn't initialized properly
        # this will retry every time until it works
        if not self.reader.available:
            self.reader = SensorShmReader()
            if not self.reader.available:
                self.get_logger().warning("Not connected to shared memory. Is shm writer running?")
                return

        snap = self.reader.read_snapshot_dict()

        if snap:
            # add ros timestamp so subscribers can compute delay
            snap["_t_publish_ns"] = self.get_clock().now().nanoseconds
            msg = String()
            msg.data = json.dumps(snap)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")

        else:
            self.get_logger().warning("Failed to read snapshot from shared memory")

    def destroy_node(self):
        self.reader.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SpiPublisher()
    print("started publisher")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
