import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class RpmPublisherNode(Node):
    def __init__(self):
        super().__init__('rpm_publisher_node')
        self.publisher = self.create_publisher(String, '/rpm', 10)
        self.get_logger().info("rpm publisher node initialized...")
        self.ser = serial.Serial("/dev/serial0", baudrate=115200, timeout=1)




def main(args=None):
    rclpy.init(args=args)
    node = RpmPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()