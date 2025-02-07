from math import sin, cos

PI = 3.14159265359
R = 0.029 # meters
D = 0.15 # meters
V = 0.911 # say v 300*2PI/60 = rad/sec, v=w/R, V = 300*2PI/60*R


def target_wheel_rpm(theta:float, wbz:float) -> tuple:
    """return u1, u2, u3,(for linear movement) in rpm"""
    theta = PI/180 * theta # convert to radian
    wbz = -PI if wbz == -1 else PI
    vbx = sin(theta) * V
    vby = cos(theta) * V
    u1 = (-D*wbz + vbx) / R * ( 60 / (2 * PI))# how fast wheel 1 must rotate
    u2 = (-D*wbz - 0.5*vbx - 0.866*vby) / R * ( 60 / (2 * PI))
    u3 = (-D*wbz - 0.5*vbx + 0.866*vby) / R * ( 60 / (2 * PI))
    return u1, u2, u3 # return the target rpm of each wheel 

