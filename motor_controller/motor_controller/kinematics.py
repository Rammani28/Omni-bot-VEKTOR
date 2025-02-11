from math import sin, cos, pi, radians, degrees

wheel_radius = 0.0295 # meters
bot_radius = 0.075 # meters
# bot_velocity = wheel_radius * (180 * 2 * pi / 60)

rpm_factor = 60 / (2 * pi)
xla= 5
def target_wheel_rpm(theta, wz, magnitude) -> tuple[float, float, float]:
    if wz == 1:
        wz = xla
    if wz == -1:
        wz = -xla
    theta = radians(theta)
    # wz = wz/bot_radius
    # bot_velocity = wheel_radius * (180 * 2 * pi / 60)
    bot_velocity = wheel_radius * (180 * 2 * pi / 60)

    if magnitude == 0:
        bot_velocity =0

    vy = sin(theta) * bot_velocity
    vx = cos(theta) * bot_velocity


    u1 = round((-bot_radius*wz + vx) / wheel_radius * rpm_factor, 4)# how fast wheel 1 must rotate
    u2 = round((-bot_radius*wz - cos(pi/3)*vx - sin(pi/3)*vy) / wheel_radius * rpm_factor, 4)
    u3 = round((-bot_radius*wz - cos(pi/3)*vx + sin(pi/3)*vy) / wheel_radius * rpm_factor, 4)
    
    return u1, u2, u3 # return the target rpm of each wheel 
