import RPi.GPIO as GPIO
from motor_controller.utils import rpm_to_pwm

class PIDController:
    def __init__(self, Kp=0.95, Ki=0.08, Kd=0.095,  min_output=0, max_output=99):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_output = min_output
        self.max_output = max_output
        self.output = 0
        self.integral = 0
        self.derivative = 0
        self.last_error = 0
        self.error = 0

    def compute(self, target_rpm, current_rpm):
        self.error = target_rpm - current_rpm
        self.integral += self.error

        self.derivative = self.error - self.last_error
        self.last_error = self.error
        
        # Prevent integral wind-up
        # self.integral = max(min(self.integral, self.max_output / self.Ki), -self.max_output / self.Ki)
        # self.integral = 0
        
        self.output = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative
        # self.output = int(max(min(self.output, self.max_output), 0))
        # output = int(max(min(self.output, self.max_output), 0) if self.output > 0 else 0)
        print(f"op:{self.output:.2f}\t", end='')
        return self.output

class Motor:
    def __init__(self, in1=5, in2=6, name="motor1"):
        self.in1 = in1
        self.in2 = in2
        self.name = name
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        self.pwm_in1 = GPIO.PWM(self.in1, 1000) # PWM frequency 1kHz
        self.pwm_in2 = GPIO.PWM(self.in2, 1000)
        self.pwm_in1.start(0)
        self.pwm_in2.start(0)
        self.pid_controller = PIDController()

    def stop(self):
        self.pwm_in1.ChangeDutyCycle(0)
        self.pwm_in2.ChangeDutyCycle(0)

    def rotate_forward(self, pwm_value):
        self.pwm_in1.ChangeDutyCycle(pwm_value)
        self.pwm_in2.ChangeDutyCycle(0)

    def rotate_backward(self, pwm_value):
        self.pwm_in1.ChangeDutyCycle(0)
        self.pwm_in2.ChangeDutyCycle(pwm_value)
    
    def rotate(self, u_n, rpm_n):
        target_rpm = abs(u_n)
        rpm_from_pid = self.pid_controller.compute(target_rpm, rpm_n)
        error_rpm = rpm_from_pid
        pwm = rpm_to_pwm(abs(rpm_n) + error_rpm)
        print(f"pwm:{pwm:.2f}\t")
        (self.rotate_backward if u_n < 0 else self.rotate_forward)(pwm)

    def destroy(self):
        self.pwm_in1.stop()
        self.pwm_in2.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")
