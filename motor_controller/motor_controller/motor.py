import RPi.GPIO as GPIO # https://pypi.org/project/RPi.GPIO/


class PIDController:
    def __init__(self, Kp=1.0, Ki=0.1, Kd=0.05, min_output=12, max_output=100):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.error  = 0
        # self.last_error = 0
        self.min_output = min_output
        self.max_output = max_output

        self.pid_pwm = 0

    def compute(self, target_pwm, current_pwm):
        self.error = target_pwm - current_pwm
        self.integral += self.error
        # derivative = error - self.last_error
        self.last_error = self.error
        temp = self.Kp * self.error + self.Ki * self.integral # TODO remove
        output = max(min(self.Kp * self.error + self.Ki * self.integral, 100), 0) # + self.Kd * derivative
        # self.pid_pwm = output
        # print(f"target:{target_pwm:.2f}")
        # print(f"current_pwm:{current_pwm:.2f}")
        # print(f"actual_pwm to be written:{temp:.2f}")
        return output


class Motor:
    def __init__(self, in1:int=5, in2:int=6, name="motor1"):
        self.in1 = in1
        self.in2 = in2
        self.name = name
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        self.pwm_in1 = GPIO.PWM(self.in1, 2000) # pwm frequency 2kHz on pin in1
        self.pwm_in2 = GPIO.PWM(self.in2, 2000)
        self.pwm_in1.start(0)
        self.pwm_in2.start(0)
        self.direction = 1
        self.pid_controller = PIDController()
        self.last_rpm = 0
        # self.current = 0
        

    def stop(self):
        self.pwm_in1.ChangeDutyCycle(0)
        self.pwm_in2.ChangeDutyCycle(0)

    def rotate_forward(self, pwm_value=0):
        # print("rotating forward ...\n")
        self.pwm_in1.ChangeDutyCycle(pwm_value)
        self.pwm_in2.ChangeDutyCycle(0)

    def rotate_backward(self, pwm_value):
        # print("rotating backward ...\n")
        self.pwm_in1.ChangeDutyCycle(0)
        self.pwm_in2.ChangeDutyCycle(pwm_value)
    
    def rotate(self, u_n, rpm_n):
        # print(f"{self.name} rotating")
        # print(f"{self.name}rpm_n got inside rotate:{rpm_n}")
        # if rpm_n is not None:
        #     self.last_rpm = rpm_n
        # current = self.last_rpm if rpm_n is None else rpm_n
        # current = rpm_n
        target = abs(u_n)
        pwm = self.pid_controller.compute(target, rpm_n)
        (self.rotate_backward if u_n < 0 else self.rotate_forward)(pwm)
        # print(f"{self.name} pwm1 written:{pwm:.2f}")

        # above is same as below
        # if rpm_n is None:
        #     pwm = self.pid_controller.compute(abs(u_n), self.last_rpm)
        #     if u_n<0:
        #         self.rotate_backward(pwm)
        #     else:
        #         self.rotate_forward(pwm)
        # else:
        #     self.last_rpm = rpm_n
        #     pwm = self.pid_controller.compute(target_pwm=abs(u_n), current_pwm=rpm_n)
        #     if u_n<0:
        #         self.rotate_backward(pwm)
        #     else:
        #         self.rotate_forward(pwm)

    def destroy(self):
        self.pwm_in1.stop()
        self.pwm_in2.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")