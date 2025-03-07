import rclpy
from rclpy.node import Node
import serial
from wheel_sensors_interfaces.msg import WheelRPMs, OdomData
import time

class RpmReaderNode(Node):
    def __init__(self):
        super().__init__('rpm_reader')
        self.uart_port = serial.Serial("/dev/serial0", 115200, timeout=0.1)
        print(self.uart_port)
        self.publisher_rpm = self.create_publisher(WheelRPMs, '/bot/current_rpms', 10)
        self.publisher_odom = self.create_publisher(OdomData, '/bot/odometry', 10)
        self.timer = self.create_timer(100/1000, self.publish_rpm) # 50ms
        self.get_logger().info("uart rpm odom reader node started.")
        self.old_log_msg = ''

    def publish_rpm(self):
        try:
            rpm_data = self.uart_port.readline().decode().strip()
            if rpm_data:
                rpm_values = rpm_data.split(',')
                msg_rpm = WheelRPMs()
                msg_odom = OdomData() 
                msg_rpm.rpm1 = float(rpm_values[0])
                msg_rpm.rpm2 = float(rpm_values[1])
                msg_rpm.rpm3 = float(rpm_values[2])
                msg_odom.x = float(rpm_values[3])
                msg_odom.y = float(rpm_values[4])
                msg_odom.pose = float(rpm_values[5])

                self.publisher_rpm.publish(msg_rpm)
                self.publisher_odom.publish(msg_odom)
                new_log_msg= ('\n'
                    f'rpms:{msg_rpm.rpm1}, {msg_rpm.rpm2}, {msg_rpm.rpm3}\n'
                    f'x: {msg_odom.x}, y: {msg_odom.y}, pose: {msg_odom.pose}'
                    )
                if new_log_msg != self.old_log_msg:
                    self.get_logger().info(new_log_msg)
                    self.old_log_msg = new_log_msg
            # else:
                # print('no rpm data from serial usb port')
        except Exception as e:
            self.get_logger().warning(f'Error reading RPM data: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RpmReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()