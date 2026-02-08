import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import urllib.request

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.subscription = self.create_subscription(
            Int32,
            'servo_angle',
            self.angle_callback,
            10
        )
        self.esp32_ip = '192.168.1.53'  # 改成你ESP32的IP
        self.get_logger().info('ESP32 Bridge node started')

    def angle_callback(self, msg):
        angle = msg.data
        if 0 <= angle <= 180:
            try:
                url = f'http://{self.esp32_ip}/servo?angle={angle}'
                urllib.request.urlopen(url, timeout=1)
                self.get_logger().info(f'Sent angle {angle} to ESP32')
            except Exception as e:
                self.get_logger().warn(f'Failed to reach ESP32: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

