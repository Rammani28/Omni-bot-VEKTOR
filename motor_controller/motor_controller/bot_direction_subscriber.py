import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from motor_controller.kinematics import target_wheel_rpm
from motor_controller.recorder import Recorder
from motor_controller.motor2 import Motor
from motor_controller.motor2 import PIDController
import threading

# corr_factor_1 = 0.85
# corr_factor_2 = 0.95
# corr_factor_3 = 0.955

kp_1 = 0.9
ki_1 = 0.6857
kd_1 = 0.000

ser = serial.Serial("/dev/serial0", baudrate=115200, timeout=0)
recorder = Recorder()

class PwmSubscriberNode(Node):
    def __init__(self):
        super().__init__('bot_direction_subscriber') # name of node
        print("subscriber initialized, !!!!!!!RUN CODE ON THONNY!!!!!!!!!!")

        self.subscription = self.create_subscription(msg_type=String, topic='/bot_direction', callback=self.listener_callback, qos_profile=10) # subscribe topic /led_control using String type, queue size is 10
        self.motor1 = Motor(6, 5, name='motor1') # physical 29, 31 
        self.motor1.pid_controller = PIDController(Kp=0.9, Ki=0.6857, Kd=kd_1)

        self.motor2 = Motor(17, 27, 'motor2')  # physical 11, 13
        self.motor2.pid_controller = PIDController(Kp=kp_1, Ki=ki_1, Kd=kd_1)

        self.motor3 = Motor(24, 23, 'motor3') # physical 16 18
        self.motor3.pid_controller = PIDController(kp_1, ki_1, kd_1)


    def stop_motors(self):
        self.motor1.stop()
        self.motor2.stop()
        self.motor3.stop()

    def listener_callback(self, msg):
        theta = float(msg.data.split(',')[0]) 
        wbz = float(msg.data.split(',')[1])
        magnitude = float(msg.data.split(',')[2])
        start = True
        if magnitude == 0 and wbz == 0:
            start = False
            self.stop_motors()
            return

        if start:
            rpms_pico = ser.readline().decode().strip()  # Read and decode
            if rpms_pico:
                print("-------------------------------")
                print("angle", theta, wbz)
                try:
                    rpm1, rpm2, rpm3 = [float(rpm) for rpm in rpms_pico.split(',')] #is in pwm value 
                    print(f"r1:{rpm1:.2f}\t r2:{rpm2:.2f}\t r3:{rpm3:.2f}")
                except ValueError:
                    print("\n\nmissed an rpm measurement\n\n")
                    # self.stop_motors()
                    return

                u1, u2, u3 = target_wheel_rpm(theta, wbz, magnitude)
                print(f"u1:{u1:.2f}\t u2:{u2:.2f}\t u3:{u3:.2f}")


                # t1 = threading.Thread(target=self.motor1.rotate, args=(u1, rpm1))
                # t2 = threading.Thread(target=self.motor2.rotate, args=(u2, rpm2))
                # t3 = threading.Thread(target=self.motor3.rotate, args=(u3, rpm3))

                # t1.start()
                # t2.start()
                # t3.start()

                # t1.join()
                # t2.join()
                # t3.join()
                recorder.record(u3,rpm3)


                self.motor1.rotate(u1, rpm1)
                self.motor2.rotate(u2, rpm2)
                self.motor3.rotate(u3, rpm3)
                print()






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
        # recorder.print_record()
        recorder.print_record(kp_1, ki_1, kd_1)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()