import utime
from machine import Pin

IRQ_RISING_FALLING = Pin.IRQ_RISING | Pin.IRQ_FALLING

_FLAG = const(0x10)

# Rotary Encoder States
_R_START = const(0x0)
_R_CW_1 = const(0x1)
_R_CW_2 = const(0x2)
_R_CW_3 = const(0x3)
_R_CCW_1 = const(0x4)
_R_CCW_2 = const(0x5)
_R_CCW_3 = const(0x6)
_R_ILLEGAL = const(0x7)

_transition_table = [

    # |------------- NEXT STATE -------------|            |CURRENT STATE|
    # CLK/DT    CLK/DT     CLK/DT    CLK/DT
    #   00        01         10        11
    [_R_START, _R_CCW_1, _R_CW_1,  _R_START],             # _R_START
    [_R_CW_2,  _R_START, _R_CW_1,  _R_START],             # _R_CW_1
    [_R_CW_2,  _R_CW_3,  _R_CW_1,  _R_START],             # _R_CW_2
    [_R_CW_2,  _R_CW_3,  _R_START, _R_START | _FLAG],     # _R_CW_3
    [_R_CCW_2, _R_CCW_1, _R_START, _R_START],             # _R_CCW_1
    [_R_CCW_2, _R_CCW_1, _R_CCW_3, _R_START],             # _R_CCW_2
    [_R_CCW_2, _R_START, _R_CCW_3, _R_START | _FLAG],     # _R_CCW_3
    [_R_START, _R_START, _R_START, _R_START]]             # _R_ILLEGAL

_transition_table_half_step = [
    [_R_CW_3,            _R_CW_2,  _R_CW_1,  _R_START],
    [_R_CW_3 | _FLAG, _R_START, _R_CW_1,  _R_START],
    [_R_CW_3 | _FLAG,  _R_CW_2,  _R_START, _R_START],
    [_R_CW_3,            _R_CCW_2, _R_CCW_1, _R_START],
    [_R_CW_3,            _R_CW_2,  _R_CCW_1, _R_START | _FLAG],
    [_R_CW_3,            _R_CCW_2, _R_CW_3,  _R_START | _FLAG],
    [_R_START,           _R_START, _R_START, _R_START],
    [_R_START,           _R_START, _R_START, _R_START]]

_STATE_MASK = const(0x07)
_FLAG_MASK = const(0x10)

class RotaryEncoder():
    def __init__(self, pin_num_clk, pin_num_dt, ppr=20, reverse=False, half_step=False, timeout = 100000):
        self.pin_clk = Pin(pin_num_clk, Pin.IN)
        self.pin_dt = Pin(pin_num_dt, Pin.IN)
        self.pin_clk.irq(self.process_rotary_pins, IRQ_RISING_FALLING)
        self.pin_dt.irq(self.process_rotary_pins, IRQ_RISING_FALLING)
        
        self.ppr = ppr
        self.timeout = timeout #in us
        self.reverse = -1 if reverse else 1
        self.value = 0
        self.state = _R_START
        self.half_step = half_step
        self.last_timestamp = None
        self.current_timestamp = None
        self.rpm = 0
        self.rpm_factor = 60/self.ppr

    def deinit(self):
        self.pin_clk.irq(handler=None)
        self.pin_dt.irq(handler=None)
        
    def process_rotary_pins(self, pin):
        clk_dt_pins = (self.pin_clk.value() << 1) | self.pin_dt.value()
            
        # Determine next state
        if self.half_step:
            self.state = _transition_table_half_step[self.state & _STATE_MASK][clk_dt_pins]
        else:
            self.state = _transition_table[self.state & _STATE_MASK][clk_dt_pins]
        flag = self.state & _FLAG_MASK
        
        if flag == _FLAG:
            self.current_timestamp = utime.ticks_us()
            if self.last_timestamp is not None:
                dt = utime.ticks_diff(self.current_timestamp, self.last_timestamp) / 1_000_000
                
                if dt > 0:
                    self.rpm = (1/dt) * self.rpm_factor
            
            self.last_timestamp = self.current_timestamp
                
    def get_rpm(self):
        # Reset RPM if no pulse detected for longer than timeout
        if utime.ticks_diff(utime.ticks_us(), self.last_timestamp) > self.timeout:
            self.rpm = 0
            self.last_timestamp = None
        return self.rpm

