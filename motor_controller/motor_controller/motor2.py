import RPi.GPIO as GPIO

class PIDController:
    def __init__(self, Kp=1.0, Ki=0.1, min_output=12, max_output=100):
        self.Kp = Kp
        self.Ki = Ki
        self.min_output = min_output
        self.max_output = max_output
        self.integral = 0
        self.error = 0

    def compute(self, target_pwm, current_pwm):
        self.error = target_pwm - current_pwm
        self.integral += self.error
        
        # Prevent integral wind-up
        self.integral = max(min(self.integral, self.max_output / self.Ki), -self.max_output / self.Ki)
        
        output = self.Kp * self.error + self.Ki * self.integral
        output = max(min(output, self.max_output), self.min_output if output > 0 else 0)
        
        print(f"Computed PWM: {output}")
        return output

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
        target = abs(u_n)
        pwm = self.pid_controller.compute(target, rpm_n)
        (self.rotate_backward if u_n < 0 else self.rotate_forward)(pwm)

    def destroy(self):
        self.pwm_in1.stop()
        self.pwm_in2.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")
