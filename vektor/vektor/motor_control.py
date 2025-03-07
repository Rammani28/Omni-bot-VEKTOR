import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from vektor.pid_controller import PIDController
from vektor.motor import Motor
from wheel_sensors_interfaces.msg import MotorSpeeds, WheelRPMs, BotDirections, WheelDistances
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from vektor.recorder import Recorder


# KP, KI, KD = 0.9, 0.6857, 0.0
KP = [4, 6, 6] # done
KI = [0, 0, 0]
KD = [0, 0, 0]
recorder = Recorder()

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_contorl_node')
        self.target_rpms_subscriber = self.create_subscription(MotorSpeeds, '/bot/target_rpms', self.target_rpms_callback, QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.current_rpms_subscriber = self.create_subscription(WheelRPMs, '/bot/current_rpms', self.current_rpms_callback, 10)
        self.bot_state_subscriber = self.create_subscription(Bool, '/r2_pressed', self.bot_state_callback, QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.angle_subscriber = self.create_subscription(BotDirections, '/bot_directions', self.angle_callback, QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info('Motor Controller Node Initialized')

        self.r2_pressed = False
        self.angle = self.wbz = self.magnitude = 0
        self.u1 = self.u2 = self.u3 = 0
        self.rpm1 = self.rpm2 = self.rpm3 = 0
        self.old_log_msg = ''

        # self.motors = [
        #     Motor(6, 5, 'motor1'), # physical 29, 31
        #     Motor(17, 27, 'motor2'), # physical 11, 13
        #     Motor(24, 23, 'motor3') # physical 16, 18
        # ]
        self.motor1 = Motor(6, 5, 'motor1')
        self.motor2 = Motor(17, 27, 'motor2')
        self.motor3 = Motor(24, 23, 'motor3')

        # for motor in self.motors:
        #     motor.pid_controller = PIDController(KP, KI, KD)

        self.motor1.pid_controller = PIDController(KP[0], KI[0], KD[0])
        self.motor2.pid_controller = PIDController(KP[1], KI[1], KD[1])
        self.motor3.pid_controller = PIDController(KP[2], KI[2], KD[2])
        # self.timer = self.create_timer(8, callback=self.killer)

    # def killer(self): # for pid tuning testing
    #     raise KeyboardInterrupt

    def angle_callback(self, msg):
        self.angle, self.wbz, self.magnitude = msg.theta, msg.wbz, msg.magnitude
        # self.angle, self.wbz, self.magnitude = 90, 1, 0  # for pid tuning

    def bot_state_callback(self,msg):
        self.r2_pressed = msg.data # true or false
        if not self.r2_pressed:
            self.stop_all_motors()

        # self.r2_pressed = True  # for pid tuning

    def target_rpms_callback(self, msg):
        self.u1, self.u2, self.u3 = msg.u1, msg.u2, msg.u3
        # self.u1, self.u2, self.u3 = 120, 120, 120  # for pid tuning
        if all(u == 0 for u in [self.u1, self.u2, self.u3]):
            self.stop_all_motors()
        # else:
        #     self.run_motors()  # Run motors immediately on new command # this removed the delay in motor response

    def current_rpms_callback(self, msg):
        if self.rpm1 != msg.rpm1 or self.rpm2 != msg.rpm2 or self.rpm3 != msg.rpm3:
            recorder.record(self.u1, self.u2, self.u3, self.rpm1, self.rpm2, self.rpm3, KP, KI, KD)
        self.rpm1, self.rpm2, self.rpm3 = msg.rpm1, msg.rpm2, msg.rpm3
        self.run_motors()

    def run_motors(self):  # Todo: 
        # if self.r2_pressed and (self.magnitude or self.wbz) and any([self.u1, self.u2, self.u3]):
        #     # read about zip here: https://www.geeksforgeeks.org/zip-in-python/
        #     for motor, u, rpm in zip(self.motors, [self.u1, self.u2, self.u3], [self.rpm1, self.rpm2, self.rpm3]):
        #         motor.rotate(u, rpm)

        if self.r2_pressed and (self.magnitude or self.wbz) and any([self.u1, self.u2, self.u3]):
            self.motor1.rotate(self.u1, self.rpm1)
            self.motor2.rotate(self.u2, self.rpm2)
            self.motor3.rotate(self.u3, self.rpm3)

        new_log_message = (
            f"\n"
            f"ang: {self.angle:<10}wbz: {self.wbz:<10}magntud: {self.magnitude:.2f}\n"
            f"u1: {self.u1:<10}u2: {self.u2:<10}u3: {self.u3}\n"
            f"r1: {self.rpm1:<10}r2: {self.rpm2:<10}r3: {self.rpm3}\n"
            # f"dL: {self.pulse_left:<10}dR: {self.pulse_right:<10}dB: {self.pulse_back}\n"
            f"{'-'*50}\n"
        )
        if self.old_log_msg != new_log_message:
            self.get_logger().info(new_log_message)
            self.old_log_msg = new_log_message
        
    def stop_all_motors(self):
        # for motor in self.motors:
        #     motor.stop()

        self.motor1.stop()
        self.motor2.stop()
        self.motor3.stop()

    def __del__(self):  # destructor for motor class, How does it work?
        # for motor in self.motors:
        #     motor.destroy()
        self.motor1.destroy()
        self.motor2.destroy()
        self.motor3.destroy('last')

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.print_record()
        # GPIO.cleanup()
        if rclpy.ok():  #Prevent multiple shutdown calls
            node.destroy_node()
            print("Node destroyed")
            rclpy.shutdown()

if __name__ == '__main__':
    main()