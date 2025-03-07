#--------------------------------------odometry section------------------------------------------#
from rotary_irq_rp2 import RotaryIRQ
from LM393Tachometer_PIO import Tachometer
from machine import Pin, Timer, UART
import math
import time
import sys

encoder_right = RotaryIRQ(pin_num_clk=8, pin_num_dt=9, reverse=True, range_mode=RotaryIRQ.RANGE_UNBOUNDED)
encoder_left = RotaryIRQ(pin_num_clk=10, pin_num_dt=11, reverse=True, range_mode=RotaryIRQ.RANGE_UNBOUNDED)
encoder_aux = RotaryIRQ(pin_num_clk=14, pin_num_dt=15, reverse=True, range_mode=RotaryIRQ.RANGE_UNBOUNDED)

# Robot dimensions
L = 6.7 * 2  # cm distance between left and right encoders
B = 7.9      # cm distance between midpoint of left & right encoders and back encoder
R = 2.95     # cm (wheel radius)
N = 11.5     # encoder ticks per revolution
cm_per_tick = 2 * math.pi * R / N

# Initial positions
currentRightPosition = 0
currentLeftPosition = 0
currentAuxPosition = 0

oldRightPosition = 0
oldLeftPosition = 0
oldAuxPosition = 0

pos_x = 0
pos_y = 0
pos_h = 0

def odometry(): # this function takes 450 us to run on average.
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
    print(currentRightPosition,currentLeftPosition,currentAuxPosition)

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

def send_odometry(timer): #this function takes min 0.9 to max 1.3 ms to run 
    global pos_x, pos_y, pos_h
    odom_data = f"{pos_x},{pos_y},{pos_h}\n"
    #sys.stdout.write(odom_data)
    
    
#-----------------------------------rpm section-----------------------------------#

taco1 = Tachometer(16, 0)  # GP16
taco2 = Tachometer(17, 1)  # GP17
taco3 = Tachometer(18, 2)  # GP18
uart_for_rpm = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1)) #UART on GPIO 0/1

def send_rpms(timer): # this functions takes 800us or 0.8ms to run on average
    rpms_data = f"{taco1.get_rpm()},{taco2.get_rpm()},{taco3.get_rpm()}\n"
    #uart_for_rpm.write(rpms_data)
    
    #print(f"rpms:{rpms_data}")

# ---------------------------------main loop ----------------------------------------#

odometry_timer = Timer()
odometry_timer.init(period=100, mode=Timer.PERIODIC, callback=lambda t: odometry()) # callback takes 0.45ms, 10Hz

send_rpm_odom_timer = Timer()
send_rpm_odom_timer.init(period = 333, mode=Timer.PERIODIC, callback=send_rpms)


send_odometry_timer = Timer()
send_odometry_timer.init(period=1000, mode=Timer.PERIODIC, callback=send_odometry) # callback takes 1.3ms
