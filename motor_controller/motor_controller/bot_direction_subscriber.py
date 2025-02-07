import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from motor_controller.utils import target_wheel_rpm
from motor_controller.motor import Motor
from time import sleep

ser = serial.Serial("/dev/serial0", baudrate=115200, timeout=0)

class PwmSubscriberNode(Node):
    """CREATED YOUR OWN PROBLEM AND CRY"""
    def __init__(self):
        super().__init__('bot_direction_subscriber') # name of node
        print("subscriber initialized")
        self.subscription = self.create_subscription(msg_type=String, topic='/bot_direction', callback=self.listener_callback, qos_profile=10) # subscribe topic /led_control using String type, queue size is 10
        self.motor1 = Motor(5, 6, name='motor1') # physical 29, 31 
        self.motor2 = Motor(27, 17, 'motor2')  # physical 11, 13
        self.motor3 = Motor(23, 24, 'motor3') # physical 16 18


    def listener_callback(self, msg):
        theta = float(msg.data.split(',')[0]) 
        wbz = float(msg.data.split(',')[-1])
        # print(f"theta:{theta}, omega:{wbz}")
        
        if theta == 501:
            wbz = 0
            theta = 0
            self.motor1.stop()
            self.motor2.stop()
            self.motor3.stop()
            # print("no input from joystick....")
            return

        if theta != 501:
            u1, u2, u3 = target_wheel_rpm(theta, 0) # -100 to 100 for comparable pwm
            # rpm1, rpm2, rpm3 = [None, None, None]
            # print(f"u1={u1},\t u2={u2},\t u3={u3}")
            rpms_pico = ser.readline().decode().strip()  # Read and decode

            if rpms_pico:
                # print("\n")
                # self.get_logger().info(f"Received theta:{theta}, wbz:{wbz}")
                rpm1, rpm2, rpm3 = [float(rpm) for rpm in rpms_pico.split(',')] #is in pwm value 

                # self.get_logger().info(f"theta:{theta}, wbz:{wbz}")
                # self.get_logger().info(f"r1={rpm1},\t r2={rpm2},\t r3={rpm3}")
                # self.get_logger().info(f"u1:{u1:.1f}")
                # print(f"r1={rpm1:.2f},\t r2={rpm2:.2f},\t r3={rpm3:.2f}")
                print(f"angle:{theta}, wbz:{wbz}")
                print(f"u1:{u1:.2f} \t u2:{u2:.2f} \t u3:{u3:.2f}")
                self.motor1.rotate(u1, rpm1)
                self.motor2.rotate(u2, rpm2)
                self.motor3.rotate(u3, rpm3)


    def __del__(self):  # destructor for motor class, How does it work?
        self.motor1.destroy()
        self.motor2.destroy()
        self.motor3.destroy()

def main(args=None):
    rclpy.init(args=args)
    node = PwmSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()