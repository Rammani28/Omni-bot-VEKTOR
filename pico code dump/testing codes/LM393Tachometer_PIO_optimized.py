import rp2
from machine import Pin, Timer

@rp2.asm_pio(set_init=rp2.PIO.IN_LOW)
def pulse_counter():
    wrap_target()
    label("wait")
    wait(1, pin, 0)  
    wait(0, pin, 0)  
    jmp(x_dec, "wait")  # Decrement X register (counts pulses)
    wrap()

class Tachometer:
    def __init__(self, pin_num, sm_id=0, ppr=20, calc_interval=333):
        self.pin = Pin(pin_num, Pin.IN)
        self.sm_id = sm_id
        self.ppr = ppr
        self.calc_interval = calc_interval
        self.rpm = 0
        self.factor = 60 / (self.calc_interval / 1000)  # Precompute for efficiency

        # Initialize PIO State Machine
        self.sm = rp2.StateMachine(self.sm_id, pulse_counter, freq=1_000_000, in_base=self.pin)
        self.sm.exec("mov(x, invert(null))")  
        self.sm.active(1)

        # Timer for RPM Calculation
        self.timer = Timer()
        self.timer.init(period=self.calc_interval, mode=Timer.PERIODIC, callback=self.calc_rpm)

    def calc_rpm(self, timer):
        """ Efficient RPM Calculation """
        self.sm.exec("mov(isr, x); push(); mov(x, invert(null))")  # Optimized PIO calls
        self.rpm = (~self.sm.get() & 0xFFFFFFFF) / self.ppr * self.factor  # Faster bitwise operation

    def get_rpm(self):
        return self.rpm

    def deinit(self):
        """ Clean shutdown """
        self.timer.deinit()
        self.sm.active(0)
