
# LM393Tachometer
# Specs : 20 slits on encoder disk and rpm is calculated every second (1000 ms)

import rp2
from machine import Pin, Timer

#PIO Assembly Code for Pulse Counting
@rp2.asm_pio(set_init=rp2.PIO.IN_LOW)
def pulse_counter():
    wrap_target()
    label("wait")
    wait(1, pin, 0)  # Wait for pin to go HIGH
    wait(0, pin, 0)  # Wait for pin to go LOW (falling edge detection)
    jmp(x_dec, "wait")  # Decrement X register (counts pulses)
    wrap()

class Tachometer:
    def __init__(self, pin_num,sm_id=0, slits=20, calc_interval=1000): 
        self.pin = Pin(pin_num, Pin.IN)
        self.sm_id=sm_id
        self.slits = slits
        self.calc_interval = calc_interval
        #self.current_rpm = 0
        self.rpm = 0
        
        #Initialize PIO State Machine for pulse counting
        self.sm = rp2.StateMachine(self.sm_id, pulse_counter, freq=1_000_000, in_base=self.pin)
        self.sm.exec("mov(x, invert(null))")  # Reset counter
        self.sm.active(1)  # Start PIO state machine

        # Timer for RPM calculation
        self.timer = Timer()
        self.timer.init(period=self.calc_interval, mode=Timer.PERIODIC, callback=self.calc_rpm)

    def calc_rpm(self, timer):
        #Read pulse count from PIO state machine
        self.sm.exec("mov(isr, x)")  # Move counter value to ISR
        self.sm.exec("push()")  # Push ISR to FIFO
        pulse_count = 0xFFFFFFFF - self.sm.get()  # Read from FIFO
        self.sm.exec("mov(x, invert(null))")  # Reset counter
        
        #self.current_rpm = (pulse_count / self.slits) * (60 / (self.calc_interval / 1000))
        self.rpm = (pulse_count / self.slits) * (60 / (self.calc_interval / 1000))
        print(self.rpm)
        
        #using Exponential Moving Average to reduce random sensor fluctuation impact..kind of
        #alpha = 0.2
        #self.rpm = (alpha * self.current_rpm) + ((1 - alpha) * self.rpm)
        
        
    def get_rpm(self):
        return self.rpm
    
    def stop(self):
        self.timer.deinit()
        self.sm.active(0)  # Stop PIO state machine
        
taco1 = Tachometer(20)
