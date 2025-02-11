#utils.py

from math import sin, cos

PI = 3.14159265359
R = 0.029 # meters
D = 0.15 # meters
V = 0.911 # say v 300*2PI/60 = rad/sec, v=w/R, V = 300*2PI/60*R
rotate_rpm = 200


def target_wheel_rpm(theta:float, wbz:float) -> tuple:
    """return u1, u2, u3,(for linear movement) in rpm"""
    if theta == 501:
        theta = 0
    if wbz == 501:
        wbz = 0

    if wbz ==-1:
        u1, u2, u3 = (rotate_rpm, rotate_rpm, rotate_rpm)
        return u1,u2, u3
    if wbz == 1:
        u1, u2, u3 = (-rotate_rpm, -rotate_rpm, -rotate_rpm)
        return u1, u2, u3

    # wbz = -PI if wbz == -1 else PI
    wbz = 0
    theta = PI/180 * theta # convert to radian
    vbx = sin(theta) * V
    vby = cos(theta) * V

    u1 = (-D*wbz + vbx) / R * ( 60 / (2 * PI))# how fast wheel 1 must rotate
    u2 = (-D*wbz - 0.5*vbx - 0.866*vby) / R * ( 60 / (2 * PI))
    u3 = (-D*wbz - 0.5*vbx + 0.866*vby) / R * ( 60 / (2 * PI))
    return u1, u2, u3 # return the target rpm of each wheel 

def rpm_to_pwm(rpm, max_rpm= 300, min_rpm=0, max_pwm=70, min_pwm=0):
    # return max(min(rpm / max_rpm * 100, 100), 0) # It had worked before
    x = (rpm - min_rpm)/ (max_rpm - min_rpm) * (max_pwm-min_pwm)
    x =max(min(x, max_pwm), 0)
    print(f"x={x}")
    return x

# class DataLogger(:
#     def __init__(self, 'name'):
#         self.filename = 

print(target_wheel_rpm(0, 0))