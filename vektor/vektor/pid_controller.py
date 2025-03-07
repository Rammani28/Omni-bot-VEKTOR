KP, KI, KD = 0.9, 0.6857, 0.0


class PIDController:
    def __init__(self, Kp=KP, Ki=KI, Kd=KD,  min_output=0, max_output=70):
        self.Kp = Kp
        # self.Ki = Ki
        # self.Kd = Kd
        # self.min_output = min_output
        # self.max_output = max_output
        self.output = 0
        # self.integral = 0
        # self.derivative = 0
        # self.last_error = 0
        self.error = 0

    def compute(self, target_rpm, current_rpm):
        self.error = target_rpm - current_rpm
        # self.integral += self.error

        # self.derivative = self.error - self.last_error
        # self.last_error = self.error
        
        # Prevent integral wind-up
        # try:
            # self.integral = max(min(self.integral, self.max_output / self.Ki), -self.max_output / self.Ki)
        # except ZeroDivisionError:
            # print("Ki is zero")
            # pass
        
        # self.output = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative
        self.output = self.Kp * self.error
        # self.output = int(max(min(self.output, self.max_output), 0))
        # output = int(max(min(self.output, self.max_output), 0) if self.output > 0 else 0)
        # print(f"op:{self.output:.2f}\t", end='')

        return self.output
