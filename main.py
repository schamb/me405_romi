import time
import cotask
import task_share
from pyb import Pin, Timer
from time import ticks_us, ticks_add, ticks_diff, sleep
import math
from bno055 import *
from pyb import I2C, ADC
import machine

"""
CLASSES
"""

import task_share
import cotask


class TaskManager:
    def __init__(self, motorL, motorR, encL, encR, IMU):
        self.IMU = IMU
        # constants
        self.DESTINATION = 1000
        self.forward1 = 4000
        self.forward2 = 1800
        self.forward3 = 1500
        self.back_dist = 1500
        self.return_dist = 4050
        self.yaw = 2*math.pi - 0.1
        # How are we going to do velocity? A vector?
        #self.VELOCITY = 50  # rn this is set to duty cycle, but we wil change it to some sort of velocuty
        self.VELOCITY_RAD_L , self.VELOCITY_RAD_R = 10, 10
        self.SPEED = 10

        #pins for line sensor
        self.SEN_0 = Pin.cpu.B14
        self.SEN_2 = Pin.cpu.B15
        self.SEN_3 = Pin.cpu.B11
        self.SEN_4 = Pin.cpu.B12
        self.SEN_5 = Pin.cpu.B10
        self.SEN_7 = Pin.cpu.B13
        self.white = 0


        # motors and encoders
        self.motorL = motorL
        self.motorR = motorR
        self.encL = encL
        self.encR = encR

        # motor positions
        self.posR = 0
        self.posL = 0
        self.posAbs = 0  # How do we store this?

        # share and queue (not needed rn, but maybe will need once things get more complicated)
        self.share_dest = task_share.Share('i', thread_protect=True, name="Share 0")
        self.queue = task_share.Queue('l', 16, thread_protect=True, overwrite=False, name="Queue 0")

        # flags
        self.adjust_speed_flag = False  # task_speed_up
        self.print_motor_data_flag = True
        self.move_flag = False
        self.update_position_flag = False
        self.read_line_flag = False

        self.BMP = Pin(Pin.cpu.C10, Pin.IN, Pin.PULL_UP)
        self.BMP2 = Pin(Pin.cpu.A15, Pin.IN, Pin.PULL_UP)


        self.STOP = False
        self.WALL = False
        self.PHASES = ["back", "turn45", "forward1", "turn90", "forward2"]
        self.phase = "back"

        self.BLACK = False
        self.WHITE = False
        self.end_count = 0

        # create tasks
        self.create_tasks()

    def create_tasks(self):
        controller = cotask.Task(self.task_controller, priority=5, period=40, profile=True, trace=False)
        move = cotask.Task(self.task_move, priority=6, period=25, profile=True, trace=False)
        adjust_speed = cotask.Task(self.task_adjust_speed, priority=7, period=20, profile=True, trace=False)
        update_position = cotask.Task(self.task_update_position, priority=0, period=10, profile=True, trace=False)
        print_motor_data = cotask.Task(self.task_print_motor_data, priority=1, period=150, profile=True, trace=False)
        read_line = cotask.Task(self.task_read_line, priority=8, period = 30, profile=True, trace=False)
        bump = cotask.Task(self.task_bump, priority=8, period = 30, profile=True, trace=False)

        cotask.task_list.append(controller)
        cotask.task_list.append(read_line)
        cotask.task_list.append(move)
        cotask.task_list.append(adjust_speed)
        cotask.task_list.append(update_position)
        cotask.task_list.append(bump)
        #cotask.task_list.append(print_motor_data)

    def read(self, sensors):
        sensorValues = [0,0,0,0,0,0]
        maxValue = 4095

        for i, sensor in enumerate(sensors):
            sensorValues[i] = maxValue
            # make sensor line an output (drives low briefly, but doesn't matter)
            sensor.init(sensor.OUT)
            # drive sensor line high
            sensor.value(1)


        time.sleep(.00001) # charge lines for 10 us

        # record start time before the first sensor is switched to input
        # (similarly, time is checked before the first sensor is read in the
        # loop below)
        startTime = time.ticks_us()
        t = 0

        for sensor in sensors:
            # make sensor line an input (should also ensure pull-up is disabled)
            sensor.init(sensor.IN)


        while (t < maxValue):
            t = ticks_diff(ticks_us(), startTime)
            for i in range(0, len(sensors)):
                if (sensors[i].value() == 0) and (t < sensorValues[i]):
                    # record the first time the line reads low
                    sensorValues[i] = t


        Kg = .83
        scaled = [0, 0, 0, 0, 0, 0]
        scaled[0] = -15 if sensorValues[5] > 900 else 0 #PIN0
        scaled[1] = -5 if sensorValues[4] > 900 else 0 #PIN2
        scaled[2] = -1 if sensorValues[3] > 700 else 0 #PIN3
        scaled[3] = 1 if sensorValues[2] > 700 else 0 #PIN4
        scaled[4] = 5 if sensorValues[1] > 900 else 0 #PIN5
        scaled[5] = 15 if sensorValues[0] > 900 else 0 #PIN7

        if scaled.count(0) == 6:
            if self.BLACK:
                self.end_count += 1
                print(f"line count: {self.end_count}")
            self.BLACK = False
        else:
            self.BLACK = True

        if self.end_count == 22:
            self.STOP = True

        



        #print(scaled)
        # if scaled.count(0) == len(scaled):
        #     self.STOP = True






        wl = Kg*sum(scaled)
        if wl < 0:
            self.VELOCITY_RAD_L = self.SPEED + wl
            self.VELOCITY_RAD_R = self.SPEED - wl
        elif wl > 0:
            self.VELOCITY_RAD_L = self.SPEED + wl
            self.VELOCITY_RAD_R = self.SPEED - wl
        else:
            self.VELOCITY_RAD_L = self.SPEED
            self.VELOCITY_RAD_R = self.SPEED

        return sensorValues


    def run_tasks(self):
        while True:
            try:
                cotask.task_list.pri_sched()
            except KeyboardInterrupt:
                self.motorL.set_duty(0)
                self.motorR.set_duty(0)
                break

    # TASK


    def task_controller(self):
        while True:
            # if romi is not at his destination then he should move
            if not self.STOP:
                self.move_flag = True
                self.read_line_flag = True
            else:
                self.move_flag = False
                self.read_line_flag = False

            while self.WALL:
                pos = self.posAbs
                phase = self.PHASES.pop(0)
                cond = True
                angle = abs(self.IMU.euler()[0])
                self.encR.update()
                posL = self.encR.get_position()
                print(phase)
                while cond:
                    self.move_flag = True
                    if phase == "back":
                        self.VELOCITY_RAD_L = -1 * self.SPEED
                        self.VELOCITY_RAD_R = -1 * self.SPEED
                        cond = self.posAbs > (pos - self.back_dist)
                    elif phase == "forward1":
                        self.VELOCITY_RAD_L = 1 * self.SPEED
                        self.VELOCITY_RAD_R = 1 * self.SPEED
                        cond = self.posAbs < (pos + self.forward1)
                    elif phase == "forward2":
                        self.VELOCITY_RAD_L = 1 * self.SPEED
                        self.VELOCITY_RAD_R = 1 * self.SPEED
                        cond = self.posAbs < (pos + self.forward2)
                    elif phase == "turn45":
                        self.VELOCITY_RAD_L = 1 * self.SPEED
                        self.VELOCITY_RAD_R = -1 * self.SPEED
                        current_angle = abs(self.IMU.euler()[0] )
                        cond = abs(abs(current_angle - angle)) < 23
                    elif phase == "turn90":
                        self.VELOCITY_RAD_L = -1 * self.SPEED
                        self.VELOCITY_RAD_R = 1 * self.SPEED
                        self.encR.update()
                        current_angle = self.encR.get_position()
                        print(f"{current_angle - posL}")
                        cond =  (current_angle - posL < 600)

                    yield

                if len(self.PHASES) == 0:
                    print("done with wall")
                    self.WALL = False
                    self.STOP = False
                    self.BLACK = False
                yield

            yield

    def task_bump(self):
        bmp = True
        while True:
            while bmp:
                #print(self.BMP.value())
                if self.BMP.value() == 0:# or self.BMP2.value() == 0:
                    self.STOP = True
                    self.WALL = True
                    bmp = False
                yield
            yield

    def task_move(self):
        while True:
            while self.move_flag:
                # speed up/slow down if we are going the wrong velocity
                if self.encL.get_velocity() != self.VELOCITY_RAD_L or self.encR.get_velocity() != self.VELOCITY_RAD_R:
                    self.adjust_speed_flag = True
                else:
                    self.adjust_speed_flag = False

                # start task for updating position
                self.update_position_flag = True
                yield

            self.motorL.set_duty(0)
            self.motorR.set_duty(0)
            self.adjust_speed_flag = False
            self.update_position_flag = False
            yield

    def task_adjust_speed(self):
        # takes in desired velocity for both motors
        while True:
            while self.adjust_speed_flag:
                dutyR, dutyL = self.get_new_duty(self.VELOCITY_RAD_L, self.VELOCITY_RAD_R)  # this is what we would calculate with the IMU
                self.motorR.set_duty(dutyR)
                self.motorL.set_duty(dutyL)

                yield

            yield

    def task_update_position(self):
        while True:
            while self.update_position_flag:
                self.encL.update()
                self.encR.update()

                self.posR = self.encR.get_position()
                self.posL = self.encL.get_position()
                self.posAbs = self.get_absolute_position()

                self.print_motor_data_flag = True
                yield

            yield

    def task_print_motor_data(self):
        while True:
            while self.print_motor_data_flag:
                print(f"{self.encL.get_velocity()} {self.encR.get_velocity()} {self.posAbs}")
                self.print_motor_data_flag = False
            yield

    def task_read_line(self):
        while True:
            while self.read_line_flag:

                vals = self.read([self.SEN_0, self.SEN_2, self.SEN_3, self.SEN_4, self.SEN_5, self.SEN_7])
                yield

            yield

    # FUNC
    # TODO functions
    def go_back(self, e_ticks):
        print("going back")
        self.move_flag = False

        start = self.posAbs
        self.VELOCITY_RAD_L = -.5 * self.SPEED
        self.VELOCITY_RAD_R = -.5 * self.SPEED
        self.move_flag = True

        return
        while self.posAbs < start + e_ticks:
            self.posR = self.encR.get_position()
            self.posL = self.encL.get_position()
            self.posAbs = self.get_absolute_position()

        return

        self.move_flag = False



    def get_new_duty(self, vleft, vright):
        # function to calculate the new duty cycles of the motors

        # Right now returs the preset duty cycle
        # Should calculate and return the updated duty cycle for each motor
        # to be set to to reach specified velocity
        velocityLreal = self.encL.get_velocity()
        velocityRreal = self.encR.get_velocity()

        L = 0
        R = 0

        Kpr = 3.8 #%s/rad
        Kir = 0.6 #%/rad
        Kdr = 0.04736 #%s2/rad

        Kpl = 3.06  # %s/rad
        Kil = 0.6 # %/rad
        Kdl = 0.0722  # %s2/rad

        #Feedback control for L Motor
        self.encL.update()
        ERRl = vleft - velocityLreal
        Fpl = Kpl*ERRl #%
        Fil = Kil*ERRl*(self.encL.deltat/1000000) #%
        Fdl = Kdl*ERRl/(self.encL.deltat/1000000) #%
        L = Fpl #+ Fil + Fdl

        # Feedback control for R Motor
        self.encR.update()
        ERRr = vright - velocityRreal
        Fpr = Kpr * ERRr  # %
        Fir = Kir * ERRr * self.encR.deltat/1000000  # %
        Fdr = Kdr * ERRr / (self.encR.deltat/1000000)  # %
        R = Fpr #+ Fir + Fdr

        return R, L

    def get_absolute_position(self):
        return (self.posL + self.posR) / 2

    def get_wheel_velocity(self, times):
        rw = 2.76 #wheel radius, inches
        wromi = 5.55 #Chasi Diameter, inches
        D = 2*24 #circle diameter, inches

        C = D*math.pi #Circumfrance, inches
        Omegay = 2*math.pi/times #Overal rotational velocity, rad/s
        V = C/times #Centerline velocity, in/s
        vright = (V - Omegay*wromi)/ rw
        vleft = (2*V/rw) - vright

        return (vleft*2.4), (vright*2.4)


# Encoder class to read and update the encoder ticks
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

    # Update Encoder Values
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

    # Geting the updated positon
    def get_position(self):
        return self.position

    # Get updated delta
    def get_delta(self):
        return self.delta

    # Define Zero Location for Linear Posiition
    def zero(self):
        self.current_val = 0
        self.old_val = 0
        self.delta = 0
        self.position = 0

    def get_velocity(self):
        self.velocity = (self.delta * math.pi) / (self.deltat * 720 / 1000000)  # retunrs rad/s
        return self.velocity

    # Gets position of each motor and prints it when able
    def task_get_position(self, shares):
        the_share, the_queue = shares
        while True:
            self.update()
            if not the_queue.full():
                the_queue.put(self.get_position())
            yield 0


# Motor driver class
class Motor:
    # Intiilization
    def __init__(self, tim, phase, enable, sleep):
        self.PH = phase
        self.tim = tim
        self.sleep = sleep
        self.EN = self.tim.channel(1, mode=Timer.PWM, pin=enable)
        self.EN.pulse_width_percent(0)
        self.sleep.high()
        self.duty = 0

    # Sets speed of each motor/wheel
    def set_duty(self, pwm):
        self.duty = pwm
        if pwm > 0:
            self.PH.low()
            self.EN.pulse_width_percent(pwm)
        elif pwm < 0:
            self.PH.high()
            self.EN.pulse_width_percent(abs(pwm))
        else:
            self.EN.pulse_width_percent(0)

    # Enable the motor
    def enable(self):
        self.sleep.high()

    # Disable the Motor
    def disable(self):
        self.sleep.low()

    def task_run_motor(self, shares):
        while True:
            the_share, the_queue = shares
            pos = the_queue.get()
            the_queue.put(pos)
            if (pos < 30000):
                if self.duty == 0:
                    self.set_duty(50)
                yield
            else:
                self.set_duty(0)
                yield


class BNO055_2(BNO055):
    def __init__(self, i2c, address=0x28, crystal=True, transpose=(0, 1, 2), sign=(0, 0, 0)):
        super().__init__(i2c, address, crystal, transpose, sign)
        self.curr = time.time_ns()
        self.angle_prev = self.euler()[0]
        self.prev = self.curr

    def get_heading_velocity(self):
        self.prev = self.curr
        self.curr = time.time_ns()

        time_interval = time.ticks_diff(self.curr, self.prev)
        angle_curr = self.euler()[0]
        angle_diff = 2 * math.pi * (angle_curr - self.angle_prev)
        self.angle_prev = angle_curr

        angle_velocity = angle_diff / time_interval
        return angle_velocity

    def get_yaw_velocity(self):
        return -1 * self.get_heading_velocity()


"""
FUNCTIONS
"""


# Set up the motors in accordance with the Motor class
def initialize_motors():
    # Right Motor
    timer_R = Timer(8, freq=20000)
    phase_R = Pin(Pin.cpu.B5, mode=Pin.OUT)
    enable_R = Pin(Pin.cpu.C6, mode=Pin.OUT)
    sleep_R = Pin(Pin.cpu.B4, mode=Pin.OUT)
    motor_R = Motor(timer_R, phase_R, enable_R, sleep_R)
    # Left Motor
    timer_L = Timer(2, freq=20000)
    phase_L = Pin(Pin.cpu.A1, mode=Pin.OUT)
    enable_L = Pin(Pin.cpu.A0, mode=Pin.OUT)
    sleep_L = Pin(Pin.cpu.A4, mode=Pin.OUT)
    motor_L = Motor(timer_L, phase_L, enable_L, sleep_L)
    return motor_R, motor_L


# Sets up Romi encoders in accorance with the Encoder Class
def initialize_encoders():
    tim_R = Timer(1, period=65535, prescaler=0)
    enc_R = Encoder(tim_R, Pin.cpu.A8, Pin.cpu.A9, 1, 2)  # A8 and A9
    tim_L = Timer(3, period=65535, prescaler=0)
    enc_L = Encoder(tim_L, Pin.cpu.A7, Pin.cpu.A6, 1, 2)
    return enc_R, enc_L


# Creates Tasks and appends them to schedualing list
def create_tasks(enc_R, enc_L):
    share0 = task_share.Share('b', thread_protect=True, name="Share 0")
    q = task_share.Queue('l', 16, thread_protect=True, overwrite=False, name="Queue 0")
    # Input 2 Zeros for R/L motor initial position
    # task for printing to console
    print_positions_task = cotask.Task(task_print_motor_data, name='Print Motor Data', priority=0, period=100,
                                       profile=True, trace=False, shares=(share0, q))
    # task for reading right motor position adn add to queue
    readR_task = cotask.Task(enc_R.task_get_position, name='Read Motor Right', priority=2, period=100,
                             profile=True, trace=False, shares=(share0, q))
    # task for reading left motor position and add to queue
    readL_task = cotask.Task(enc_L.task_get_position, name='Read Motor Left', priority=2, period=100,
                             profile=True, trace=False, shares=(share0, q))
    # tasks for running each motor
    Run_Motor_R = cotask.Task(motor_R.task_run_motor, name="Run Motor R", priority=1, period=100, profile=True,
                              trace=False, shares=(share0, q))
    Run_Motor_L = cotask.Task(motor_L.task_run_motor, name="Run Motor L", priority=1, period=100,
                              profile=True, trace=False, shares=(share0, q))
    # Add tasks to task_list
    cotask.task_list.append(print_positions_task)
    cotask.task_list.append(Run_Motor_R)
    cotask.task_list.append(Run_Motor_L)
    cotask.task_list.append(readR_task)
    cotask.task_list.append(readL_task)


# Function that runs the tasks? Can't we just put this in main?
def run_tasks():
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            break


"""
TASKS
"""


# This tasks prints the motor data continuously
def task_print_motor_data(shares):
    the_share, the_queue = shares
    while True:
        if the_queue.empty():
            print("empty queue")
        else:
            print(the_queue.get(), the_queue.get())
        yield 0


if __name__ == '__main__':
    motorR, motorL = initialize_motors()
    encR, encL = initialize_encoders()
    i2c = machine.I2C(1)
    IMU = BNO055_2(i2c)

    tm = TaskManager(motorL, motorR, encL, encR, IMU)
    tm.run_tasks()
