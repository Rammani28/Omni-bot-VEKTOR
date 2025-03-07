from machine import Pin, Timer, UART
from LM393Tachometer_PIO import Tachometer
from rotary_irq_rp2 import RotaryIRQ
from time import sleep
import time
import sys

uart1 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1)) #UART on GPIO 0/1

# Create instances for 3 tachometers
taco1 = Tachometer(16, 0)  # GP16
taco2 = Tachometer(17, 1)  # GP17
taco3 = Tachometer(18, 2)  # GP18

rencl = RotaryIRQ(
    pin_num_clk=8,  # GPIO pin for CLK
    pin_num_dt=9,   # GPIO pin for DT
    min_val=0,       # Minimum value
    max_val=100,     # Maximum value
    reverse=True,   # Direction of rotation
    range_mode=RotaryIRQ.RANGE_UNBOUNDED,  # Counting mode)
    half_step=False
)

rencr = RotaryIRQ(
    pin_num_clk=10,
    pin_num_dt=11,
    min_val=0,
    max_val=100,
    reverse=True,
    range_mode=RotaryIRQ.RANGE_UNBOUNDED,
    half_step=False
)

rencb = RotaryIRQ(
    pin_num_clk=14,
    pin_num_dt=15,
    min_val=0,
    max_val=100,
    reverse=True,
    range_mode=RotaryIRQ.RANGE_UNBOUNDED,
    half_step=False
    
)

sleep(1) # to allow pico to load all files the first time

def send_rpms(timer):
    rpms_data = f"{taco1.get_rpm()},{taco2.get_rpm()},{taco3.get_rpm()}\n"
    uart1.write(rpms_data)
    #print(f"rpms:{rpms_data}")
    
def send_distance(timer2):
    distance_data = f"{rencl.value()},{rencr.value()},{rencb.value()}\n"
    sys.stdout.write(distance_data)
    #print(f"distances:{distance_data}")
    
#def send_data(timer)
  #  data = f"{taco1.get_rpm()},{taco2.get_rpm()},{taco3.get_rpm()},{rencl.value()},{rencr.value()},{rencb.value()}\n"
 #   uart1.write(data)
    
def reset_pulses():
    rencl.reset()
    rencr.reset()
    rencb.reset()
    print("Tachometers reset")
    
def check_serial(timer3):
    if uart1.any():
        command = uart1.read().decode().strip()  # ✅ Read and decode command
        if command=='RESET_PULSES':
            reset_pulses()


pulse_reset_timer = Timer()
send_rpms_timer = Timer()
send_distance_timer = Timer()

pulse_reset_timer.init(period=1000, mode=Timer.PERIODIC, callback=check_serial)
send_rpms_timer.init(period=333, mode=Timer.PERIODIC, callback=send_rpms)
send_distance_timer.init(period=333, mode=Timer.PERIODIC, callback=send_distance)


