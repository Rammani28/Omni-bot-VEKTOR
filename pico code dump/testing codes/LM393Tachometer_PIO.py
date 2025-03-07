# LM393Tachometer
# Specs : 20 PPR, RPM calculated on a timer, no directional data

import rp2
from machine import Pin, Timer

#PIO Assembly Code for Pulse Counting
@rp2.asm_pio(set_init=rp2.PIO.IN_LOW)

def pulse_counter():
    """A PIO (assembly) code that uses the decrement operation in the jump operation,
    to essentially count the number of pulses received at specified pin, pi pico's PIO
    does not offer any arithmetic operators as it wasn't designed for it.
    
    it registers a max value (all ones) to reg x, which is decremented when jumping,
    this is then sent to ISR, pushed onto the FIFO buffer and then used in the micropython
    code using the state machines get() method
    
    You can find documentation for the rp2 library
    @ https://docs.micropython.org/en/latest/library/rp2.html"""
    
    wrap_target()
    label("wait")
    wait(1, pin, 0)  # Wait for pin to go HIGH
    wait(0, pin, 0)  # Wait for pin to go LOW (falling edge detection)
    jmp(x_dec, "wait")  # Decrement X register (counts pulses)
    wrap()

class Tachometer:
    def __init__(self, pin_num, sm_id=0, ppr=20, calc_interval=1000):
        """
        pin_num : D0 pin of LM393 Tachometer.
        sm_id : id for the state machine, assign different if using multiple instances
        ppr : pulse per revolution, basically number of slits
        calc_interval : controlls how frequently rpm is updated, in ms
        
        """
        self.pin = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        self.sm_id=sm_id
        self.ppr = ppr
        self.calc_interval = calc_interval
        self.rpm = 0
        self.current_rpm = 0
        
        #Initialize PIO State Machine for pulse counting
        self.sm = rp2.StateMachine(self.sm_id, pulse_counter, freq=1_000_000, in_base=self.pin)
        self.sm.exec("mov(x, invert(null))")  # Reset counter
        self.sm.active(1)  # Start PIO state machine

        # Timer for RPM calculation
        self.timer = Timer()
        self.timer.init(period=self.calc_interval, mode=Timer.PERIODIC, callback=self.calc_rpm)

    def calc_rpm(self, timer):
        """
        Basically periodically do the whole ISR -> FIFO Buffer -> out then calculation of RPM
        every 1 second, updating the self.rpm value
        """
        #Read pulse count from PIO state machine
        self.sm.exec("mov(isr, x)")  # Move counter value to ISR
        self.sm.exec("push()")  # Push ISR to FIFO
        pulse_count = 0xFFFFFFFF - self.sm.get()  # Read from FIFO
        self.sm.exec("mov(x, invert(null))")  # Reset counter
        
        #self.current_rpm = (pulse_count / self.slits) * (60 / (self.calc_interval / 1000))
        self.rpm = (pulse_count / self.ppr) * (60 / (self.calc_interval / 1000))
        
        #using Exponential Moving Average to reduce random sensor fluctuation impact..kind of
        #alpha = 0.2
        #self.rpm = (alpha * self.current_rpm) + ((1 - alpha) * self.rpm)
        
        
    def get_rpm(self):
        return self.rpm
    
    def deinit(self):
        """
        Deinit code to cleanly terminate the code
        """
        self.timer.deinit()
        self.sm.active(0)  # Stop PIO state machine

