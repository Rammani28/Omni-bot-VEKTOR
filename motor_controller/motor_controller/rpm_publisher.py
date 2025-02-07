import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class RPMPublisherNode(Node):
    def __init__(self):
        super().__init__('rpm_publisher')
        self.publisher_ = self.create_publisher(String, '/rpms', 10)  # Topic name
        self.serial_port = serial.Serial("/dev/serial0", baudrate=115200, timeout=0.1)
        self.timer = self.create_timer(0.1, self.publish_rpm)  # 10 Hz update rate
        
    def publish_rpm(self):
        try:
            rpm_data = self.serial_port.readline().decode().strip()
            if rpm_data:
                msg = String()
                msg.data = rpm_data
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published RPMs: {rpm_data}')
        except Exception as e:
            self.get_logger().error(f'Error reading RPM data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RPMPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
