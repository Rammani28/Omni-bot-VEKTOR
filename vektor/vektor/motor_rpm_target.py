import rclpy
from rclpy.node import Node
from vektor.kinematics import target_wheel_rpm
from wheel_sensors_interfaces.msg import BotDirections, MotorSpeeds

class WheelTargetRPMs(Node):
    def __init__(self):
        super().__init__("wheel_target_rpms_node")
        self.subscriber = self.create_subscription(BotDirections, '/bot_directions', self.bot_direction_callback, 10)
        self.publisher = self.create_publisher(MotorSpeeds, '/bot/target_rpms', 10)
        self.get_logger().info("wheel_target_rpms_node initialized...")
        self.old_log_msg = ''
    
    def bot_direction_callback(self, msg):
        motor_speeds = MotorSpeeds()
        motor_speeds.u1, motor_speeds.u2, motor_speeds.u3 = target_wheel_rpm(msg.theta, msg.wbz, msg.magnitude)
        self.publisher.publish(motor_speeds)

        new_log_msg=f"u1={motor_speeds.u1}, u2={motor_speeds.u2}, u3={motor_speeds.u3}"
        if new_log_msg != self.old_log_msg:
            self.get_logger().info(new_log_msg)
            self.old_log_msg = new_log_msg

    
def main(args=None):
    rclpy.init(args=args)
    node = WheelTargetRPMs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
