import matplotlib.pyplot as plt

class Recorder:
    def __init__(self):
        self.l_u1 = []
        self.l_u2 = []
        self.l_u3 = []
        self.l_r1 = []
        self.l_r2 = []
        self.l_r3 = []
        self.time = []
        self.kp, self.ki, self.kd = 0, 0, 0

    def record(self, u1, u2, u3, r1, r2, r3, kp, ki, kd):
        self.l_u1.append(u1)
        self.l_u2.append(u2)
        self.l_u3.append(u3)
        self.l_r1.append(r1)
        self.l_r2.append(r2)
        self.l_r3.append(r3)
        self.kp, self.ki, self.kd = kp, ki, kd

        if len(self.l_r1) == 1:  # Only create time list once
            self.time = list(range(len(self.l_r1)))

    def print_record(self):
        print(f"{'-'*40}")
        print('\n\n\n')
        print(f"kp, ki, kd = {self.kp}, {self.ki}, {self.kd}")
        print(f"u1={self.l_u1}")
        print(f"u2={self.l_u2}")
        print(f"u3={self.l_u3}")
        print(f"r1={self.l_r1}")
        print(f"r2={self.l_r2}")
        print(f"r3={self.l_r3}\n")
        print("plot_pid_response(u1, u2, u3, r1, r2, r3, kp, ki, kd)")
        print(f"\n\n{'-'*40}")
        print("\n")
