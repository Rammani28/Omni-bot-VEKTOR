from machine import Pin, Timer
import utime

class PeriodTachometer:
    def __init__(self, pin_num, ppr=20, alpha=0.1):
        self.pin = Pin(pin_num, Pin.IN, Pin.PULL_DOWN)
        self.ppr = ppr
        self.alpha = alpha
        self.last_time = None     
        self.rpm = 0 
        
        # Configure the pin to trigger an interrupt on the rising edge.
        self.pin.irq(trigger=Pin.IRQ_RISING, handler=self.pulse_handler)
    
    def pulse_handler(self, pin):
        
        current_time = utime.ticks_us()  # Get current time in microseconds
        if self.last_time is not None:
            dt = utime.ticks_diff(current_time, self.last_time)  # dt in microseconds
            if dt > 0:
                dt_sec = dt / 1_000_000  # Convert microseconds to seconds
                new_rpm = 60 / (self.ppr * dt_sec)
                
                # Exponential Moving Average
                self.rpm = (self.alpha * new_rpm) + ((1 - self.alpha) * self.rpm)
                
        self.last_time = current_time  # Update the last pulse time

    def get_rpm(self):
        return self.rpm
    
tacho1 = PeriodTachometer(16)

def send_rpm(timer):
    print(tacho1.get_rpm())


timer = Timer()
timer.init(period=2000, mode=Timer.PERIODIC, callback=send_rpm)







