import time
from pyb import Timer
from time import ticks_diff
import math
from bno055 import *

## A class for the Romi encoders.
class Encoder:
    # Initialize Encoders
    def __init__(self, tim, pin_A, pin_B, ch1, ch2):
        self.zero()
        self.tim = tim
        self.pin_A = pin_A
        self.pin_B = pin_B
        # Set timer channels and pins accordingly
        self.tim.channel(ch1, pin=pin_A, mode=Timer.ENC_AB)
        self.tim.channel(ch2, pin=pin_B, mode=Timer.ENC_AB)
        self.current_time = time.ticks_us()

        self.update()

    ## Updates encoder values.
    def update(self):
        self.old_val = self.current_val  # Reset old value
        self.old_time = self.current_time
        self.current_time = time.ticks_us()
        self.current_val = self.tim.counter()  # Grab current ticks value
        self.delta = self.current_val - self.old_val  # get delta to caluclate speed
        self.deltat = ticks_diff(self.current_time, self.old_time)

        # Handle overflow or underflow
        if self.delta < -32768:
            self.delta += 65536
        elif self.delta > 32768:
            self.delta -= 65536
        # Keep track of linear position of each encoder, basically how far each encoder has gone
        self.position += self.delta

    ## Gets position of encoder.
    def get_position(self):
        return self.position

    ## Gets encoder delta. 
    def get_delta(self):
        return self.delta

    ## Defines Zero Location for Linear Posiition
    def zero(self):
        self.current_val = 0
        self.old_val = 0
        self.delta = 0
        self.position = 0

    ## Gets velocity of encoder. 
    def get_velocity(self):
        self.velocity = (self.delta * math.pi) / (self.deltat * 720 / 1000000)  # retunrs rad/s
        return self.velocity

## A class for the Romi motors.
class Motor:
    ## The constructor.
    def __init__(self, tim, phase, enable, sleep):
        self.PH = phase
        self.tim = tim
        self.sleep = sleep
        self.EN = self.tim.channel(1, mode=Timer.PWM, pin=enable)
        self.EN.pulse_width_percent(0)
        self.sleep.high()
        self.duty = 0

    ## Sets duty cycle for motor.
    # @param pwm The duty cycle the motor will be set to
    def set_duty(self, pwm: int):
        self.duty = pwm
        if pwm > 0:
            self.PH.low()
            self.EN.pulse_width_percent(pwm)
        elif pwm < 0:
            self.PH.high()
            self.EN.pulse_width_percent(abs(pwm))
        else:
            self.EN.pulse_width_percent(0)

    ## Enables motor.
    def enable(self):
        self.sleep.high()

    ## Disables motor.
    def disable(self):
        self.sleep.low()

## A class for the IMU.
#
# This class inherits the BN055 class from this github repository: https://github.com/micropython-IMU/micropython-bno055
class BNO055_2(BNO055):
    ## The constructor.
    def __init__(self, i2c, address=0x28, crystal=True, transpose=(0, 1, 2), sign=(0, 0, 0)):
        super().__init__(i2c, address, crystal, transpose, sign)
        self.curr = time.time_ns()
        self.angle_prev = self.euler()[0]
        self.prev = self.curr
    ## Gets heading velocity.
    def get_heading_velocity(self):
        self.prev = self.curr
        self.curr = time.time_ns()

        time_interval = time.ticks_diff(self.curr, self.prev)
        angle_curr = self.euler()[0]
        angle_diff = 2 * math.pi * (angle_curr - self.angle_prev)
        self.angle_prev = angle_curr

        angle_velocity = angle_diff / time_interval
        return angle_velocity
    ## Gets yaw velocity. 
    def get_yaw_velocity(self):
        return -1 * self.get_heading_velocity()

