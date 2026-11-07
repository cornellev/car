import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

class LatencyReporter(Node):
    def __init__(self):
        super().__init__("latency_reporter")
        self.sub = self.create_subscription(
            String,
            "spi_data",
            self.callback,
            10,
        )

    def callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        t_publish_ns = data.get("_t_publish_ns")
        if t_publish_ns is None:
            self.get_logger().warn("Message has no '_t_publish_ns'; cannot compute delay.")
            return
        now_ns = self.get_clock().now().nanoseconds
        delay_ns = now_ns - t_publish_ns
        delay_ms = delay_ns / 1e6
        self.get_logger().info(f"delay_ms={delay_ms:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = LatencyReporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
