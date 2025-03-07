import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

class L1PressStatusNode(Node):
    def __init__(self):
        super().__init__("joy_controlled_bot_state")
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.l1_state_publisher = self.create_publisher(Bool, '/r2_pressed', 10)
        self.get_logger().info("R2 key state node initialized")
        self.old_log_msg = ''

    def joy_callback(self, msg):
        l1_state = Bool()
        l1_state.data = msg.buttons[7] == 1 or msg.buttons[9] ==1 # true
        self.l1_state_publisher.publish(l1_state)

        new_log_msg  = "bot moving" if l1_state.data else "bot paused"
        if new_log_msg != self.old_log_msg:
            self.get_logger().info(new_log_msg)
            self.old_log_msg = new_log_msg
    
    
def main(args=None):
    rclpy.init(args=args)
    node = L1PressStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
