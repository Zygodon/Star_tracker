# PID controller for Benbo based star tracker.
# PID algorithm adapted from Wikipedia entry on PID controllers.
# Pololu Micro Metal Motor 380:1 reduction with quadrature
# (two output) shaft encoder.
# R Pi Pico using two counters, counting both rising and falling edges.
# Parameters optimised 2022-07-18, Zeigler - Nicholls tuning.

from machine import Pin,Timer, PWM
from rp2 import PIO, asm_pio, StateMachine
import utime

DUTY_CYCLE_UPPER = 65535
DUTY_CYCLE_LOWER = 1000

COUNTER_SCALE = 490 # Empirical value

# Counter for rising and falling edges
@asm_pio()    
def PIO_COUNTER():
    set(x,0)
    wrap_target()
    label('loop')
    wait(0,pin,0)
    wait(1,pin,0)
    jmp(x_dec,'nxt') # Both edges
    label('nxt')
    wait(0,pin,0)
    jmp(x_dec,'loop')
    wrap()

class SMCounter:   
    def __init__(self, smID, InputPin):
        self.counter = 0x0
        self.sm = StateMachine(smID)
        self.pin = InputPin
        self.sm.init(PIO_COUNTER,freq=125_000_000,in_base=self.pin)
        self.sm.active(1)
    
    def value(self):
        self.sm.exec('mov(isr,x)')
        self.sm.exec('push()')
        self.counter = self.sm.get()
        return  (0x100000000 - self.counter) & 0xffffffff
    
    def reset(self):
        self.sm.active(0)
        self.sm.init(PIO_COUNTER,freq=125_000_000,in_base=self.pin)
        self.sm.active(1)
        

    def __del__(self):
        self.sm.active(0)

# Process variable is counter_A.value() + counter_B.value()
counter_A = SMCounter(smID=0,InputPin=Pin(16,Pin.IN,Pin.PULL_UP))
counter_B = SMCounter(smID=0,InputPin=Pin(17,Pin.IN,Pin.PULL_UP))

# L298N terminals IN1, IN2
IN1 = Pin(3, Pin.OUT)
IN2 = Pin(2, Pin.OUT)
IN1.high() # spin reverse
IN2.low()

# Controller output is controller.duty_u16()
controller = PWM(Pin(4))
controller.freq(1000) # PWM basic frequency cycles/second
duty_cycle = DUTY_CYCLE_LOWER
controller.duty_u16(duty_cycle)

set_point = 0.85 # Chosen 2022-07-20 to give 1 turn of drive screw in 1 min 18.5s

# PID parameters
Kp = 120 # Ku = 200; Tu = 17
Ki = 14
Kd = 255
dt = 50 # ms sampling period

A0 = Kp*dt + Ki*dt + Kd/dt
A1 = -Kp - 2*Kd/dt
A2 = Kd/dt

error = [0, 0, 0]

while True:
    error[2] = error[1]
    error[1] = error[0]
    counter_A.reset()
    counter_B.reset()
    utime.sleep_ms(dt)
    process_variable = (counter_A.value() + counter_B.value())/COUNTER_SCALE # Eliminate process_variable when diagnostic print not needed
    error[0] = set_point - process_variable # counter_A.value() + counter_B.value()
    duty_cycle = int(duty_cycle + A0*error[0] + A1*error[1] + A2*error[2])
    if (duty_cycle > DUTY_CYCLE_UPPER):
       duty_cycle = DUTY_CYCLE_UPPER
    if (duty_cycle < DUTY_CYCLE_LOWER):
       duty_cycle = DUTY_CYCLE_LOWER    
    controller.duty_u16(duty_cycle)
machine.Pin(3).value(0)

