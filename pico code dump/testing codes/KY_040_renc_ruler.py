# KY-040 Rotary Encoder
# Specs : 15 PPR, RPM calculated per 1000ms, RPM data includes direction : (+) -> fwd, (-) ->rev

from machine import Pin
from math import pi

class RotaryEncoder:
    def __init__(self, CLK_pin, DT_pin, ppr=15):
        """
        CLK and DT are basically where the quadrature square waves are generated, here we
        use these for direction.
        
        """
        self.CLK = Pin(CLK_pin, Pin.IN)
        self.DT = Pin(DT_pin, Pin.IN)
        self.count = 0
        self.ppr = ppr
        self.wheel_dia = 0.059
        self.circumference = pi * self.wheel_dia
        self.distance = 0
        
        # Attach interrupt to determine direction
        self.CLK.irq(trigger=Pin.IRQ_FALLING, handler=self.counter)
        
    def counter(self, CLK):
        x = self.DT.value()
        if x == 1:
            self.count += 1
        else:
            self.count -= 1
        
    def get_distance(self):
        """
        A signed RPM value, the sign is the direction and the magnitude is the RPM value.
        """
        self.distance = (self.count / self.ppr) * self.circumference
        #self.count = 0
        return self.distance
    
    def deinit(self):
        """
        Deinit code to cleanly terminate the code
        """
        self.CLK.irq(handler=None)


