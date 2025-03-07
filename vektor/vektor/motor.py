#motor2
import RPi.GPIO as GPIO
import sys

GPIO.setmode(GPIO.BCM) # on top to prevent multiple calls

def rpm_to_pwm(rpm, max_rpm=200, min_rpm=0, max_pwm=90, min_pwm=0):
    # return max(min(rpm / max_rpm * 100, 100), 0) # It had worked before
    x = min_pwm + (rpm - min_rpm)/ (max_rpm - min_rpm) * (max_pwm-min_pwm) 
    x =max(min(x, max_pwm), min_pwm)
    # print(f"x={x}")
    return x


class Motor:
    def __init__(self, in1=6, in2=5, name="motor1", corr_factor=1):
        self.in1 = in1
        self.in2 = in2
        self.corr_factor = corr_factor
        self.name = name
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        self.pwm_in1 = GPIO.PWM(self.in1, 1000) # PWM frequency 1kHz
        self.pwm_in2 = GPIO.PWM(self.in2, 1000)
        self.pwm_in1.start(0)
        self.pwm_in2.start(0)
        self.pid_controller = None


    def stop(self):
        self.pwm_in1.ChangeDutyCycle(0)
        self.pwm_in2.ChangeDutyCycle(0)

    def rotate_forward(self, pwm_value):
        self.pwm_in1.ChangeDutyCycle(pwm_value)
        self.pwm_in2.ChangeDutyCycle(0)

    def rotate_backward(self, pwm_value):
        self.pwm_in1.ChangeDutyCycle(0)
        self.pwm_in2.ChangeDutyCycle(pwm_value)
    
    def rotate(self, u_n, rpm_n=0):
        target_rpm = abs(u_n)
        if target_rpm >= 5: # don't run pid loop if u < 5, as its already insufficient to run motor
            rpm_from_pid = self.pid_controller.compute(target_rpm, rpm_n)
            error_rpm = rpm_from_pid
            pwm = rpm_to_pwm(target_rpm + error_rpm)
            (self.rotate_backward if u_n < 0 else self.rotate_forward)(pwm)

    # def rotate(self, u_n, rpm_n = 0):
    #     target_rpm = abs(u_n)
    #     target_pwm = rpm_to_pwm(target_rpm)
        
    #     if target_rpm >= 5: # don't run pid loop if u < 5, as its already insufficient to run motor
    #         error_rpm = self.pid_controller.compute(target_rpm, rpm_n)

    #         error_pwm = rpm_to_pwm(error_rpm)

    #         pwm = target_pwm + error_pwm


    def destroy(self, index='not_last'):
        self.pwm_in1.stop()
        self.pwm_in2.stop()
        if index == 'last': # to prevent multiple cleanup calls
            GPIO.cleanup()
            print("GPIO cleaned up")

    