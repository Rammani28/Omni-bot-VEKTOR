# MIT License (MIT)
# Copyright (c) 2021 Mike Teachman
# https://opensource.org/licenses/MIT

# example for MicroPython rotary encoder

from rotary_irq_rp2 import RotaryIRQ
import time

#r = RotaryIRQ(pin_num_clk=8, pin_num_dt=9, reverse=True, range_mode=RotaryIRQ.RANGE_UNBOUNDED)
#r = RotaryIRQ(pin_num_clk=10, pin_num_dt=11, reverse=True, range_mode=RotaryIRQ.RANGE_UNBOUNDED)
r = RotaryIRQ(pin_num_clk=14, pin_num_dt=15, reverse=True, range_mode=RotaryIRQ.RANGE_UNBOUNDED)


val_old = r.value()
while True:
    val_new = r.value()

    if val_old != val_new:
        val_old = val_new
        print('result =', val_new)

    time.sleep_ms(50)