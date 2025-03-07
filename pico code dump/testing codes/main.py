#--------------------------------------odometry section------------------------------------------#
from machine import Pin, UART, Timer, soft_reset
import math
import utime
import sys
from KY_040_pulse_interval import RotaryEncoder
from rotary_irq_rp2 import RotaryIRQ

wheel_enc_1 = RotaryEncoder(pin_num_clk=21, pin_num_dt=20)
wheel_enc_2 = RotaryEncoder(pin_num_clk=19, pin_num_dt=18)
wheel_enc_3 = RotaryEncoder(pin_num_clk=17, pin_num_dt=16)

encoder_left = RotaryIRQ(pin_num_clk=10, pin_num_dt=11, reverse=True, range_mode=RotaryIRQ.RANGE_UNBOUNDED)
encoder_right = RotaryIRQ(pin_num_clk=12, pin_num_dt=13, reverse=True, range_mode=RotaryIRQ.RANGE_UNBOUNDED)
encoder_aux = RotaryIRQ(pin_num_clk=14, pin_num_dt=15, reverse=True, range_mode=RotaryIRQ.RANGE_UNBOUNDED)

# Robot dimensions
L = 6.7 * 2  # cm distance between left and right encoders
B = -7.9      # cm distance between midpoint of left & right encoders and back encoder
R = 2.9     # cm (wheel radius)
N = 15     # encoder ticks per revolution
cm_per_tick = 2 * math.pi * R / N #1.2147

#---Odom Variables-----------------------------------------------------------------------
currentRightPosition = 0
currentLeftPosition = 0
currentAuxPosition = 0

oldRightPosition = 0
oldLeftPosition = 0
oldAuxPosition = 0

pos_x = 0
pos_y = 0
pos_h = 0

#---Rpm Variables-----------------------------------------------------------------------

uart_rpm_odom = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), ) #UART on GPIO 0/1

def odometry(timer): # this function takes 450 us to run on average.
# def odometry():
    global oldRightPosition, oldLeftPosition, oldAuxPosition
    global currentRightPosition, currentLeftPosition, currentAuxPosition
    global pos_x, pos_y, pos_h

    # Store previous positions
    oldRightPosition = currentRightPosition
    oldLeftPosition = currentLeftPosition
    oldAuxPosition = currentAuxPosition

    # Read current encoder values
    currentRightPosition = encoder_right.value()
    currentLeftPosition = encoder_left.value()
    currentAuxPosition = encoder_aux.value()
    #print(currentLeftPosition, currentRightPosition, oldAuxPosition)
    
    # Calculate delta movements
    dn1 = currentLeftPosition - oldLeftPosition
    dn2 = currentRightPosition - oldRightPosition
    dn3 = currentAuxPosition - oldAuxPosition

    # Compute change in heading, forward, and sideways displacement
    dtheta = cm_per_tick * (dn2 - dn1) / L
    dx = cm_per_tick * (dn1 + dn2) / 2.0
    dy = cm_per_tick * (dn3 - (dn2 - dn1) * B / L)

    # Convert to global coordinates
    theta = pos_h + (dtheta / 2.0)
    pos_x += dx * math.cos(theta) - dy * math.sin(theta)
    pos_y += dx * math.sin(theta) + dy * math.cos(theta)
    pos_h += dtheta  # Update heading
    
def send_rpm_odom(): # this functin took less than 1 ms to run fully
#     start = utime.ticks_us()

    rpm1 = wheel_enc_1.get_rpm()
    rpm2 = wheel_enc_2.get_rpm()
    rpm3 = wheel_enc_3.get_rpm()
    data = f"{rpm1:.2f},{rpm2:.2f},{rpm3:.2f},{pos_x:.2f},{pos_y:.2f},{math.degrees(pos_h):.2f}\n" # took max 41 bytes
#     print(data)
    uart_rpm_odom.write(data)
    
#     end = utime.ticks_us()
#     print("Write_time_us:", utime.ticks_diff(end, start)) # 

#--------------------------main loop --------------------------------#
utime.sleep_ms(50)
odometry_timer = Timer()
odometry_timer.init(period=50, mode=Timer.PERIODIC, callback=odometry)
utime.sleep_ms(50) #


try:
    while True:
        #print("main loop")
        send_rpm_odom()
#         print(currentLeftPosition, currentRightPosition, oldAuxPosition)
        #measure_execution_time(calc_rpm)
        utime.sleep_ms(100)
except Exception as e:
    print(f"Error: {e}")
    soft_reset()
