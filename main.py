import time
import cotask
import task_share
from pyb import Pin, Timer
from time import ticks_us, ticks_diff
import math
from bno055 import *
import machine
from classes import BNO055_2, Encoder, Motor
import task_share
import cotask

"""
CLASSES
"""

## TaskManager class for Romi.
#
# This class schedules and manages all tasks for the Romi.
# For more details about tasks and control flow, visit [Task Breakdown](md_task__breakdown.html)
class TaskManager:
    ## Constructor
    def __init__(self, motorL, motorR, encL, encR, IMU):
        self.IMU = IMU
        # constants

        #WALL SEQUENCE
        self.forward1 = 3000
        self.forward2 = 2500
        self.back_dist = 750
        self.angle = 500

        #END SEQUENCE
        self.return_dist = 2500
        self.adjust_forward_dist = 2000
        self.white_goal = 22
    
        #SPEED CONTROL
        self.scale = 1.2
        self.SPEED = 12
        self.VELOCITY_RAD_L , self.VELOCITY_RAD_R = self.SPEED, self.SPEED
        
        #pins for line sensor and bumb sensor
        self.SEN_0 = Pin.cpu.B14
        self.SEN_2 = Pin.cpu.B15
        self.SEN_3 = Pin.cpu.B11
        self.SEN_4 = Pin.cpu.B12
        self.SEN_5 = Pin.cpu.B10
        self.SEN_7 = Pin.cpu.B13
        self.BMP = Pin(Pin.cpu.C10, Pin.IN, Pin.PULL_UP)
        self.BMP2 = Pin(Pin.cpu.A15, Pin.IN, Pin.PULL_UP)

        # motors and encoders
        self.motorL = motorL
        self.motorR = motorR
        self.encL = encL
        self.encR = encR

        # motor positions
        self.posR = 0
        self.posL = 0
        self.posAbs = 0 

        # flags
        self.adjust_speed_flag = False  # task_speed_up
        self.move_flag = False
        self.update_position_flag = False
        self.read_line_flag = False
        self.sense_line_flag = False
        self.line_sensed_flag = False
        self.black_flag = False

        #Phases
        self.STOP = False
        self.WALL = False
        self.END = False
        self.WALL_PHASES = ["back", "turn45", "forward1", "turn90", "adjust forward", "forward2", "orient"]
        self.END_PHASES = ["turn180", "line sense","forward", "stop", "stop", "stop", "stop"]

        #condition to trigger end phase
        self.end_count = 0

        # create tasks
        self.create_tasks()

    ## Creates tasks and adds them to the task list. 
    #
    # @arg @c controller
    # @arg @c move
    # @arg @c adjust_speed
    # @arg @c update_position
    # @arg @c read_line
    # @arg @c bump
    def create_tasks(self):
        controller = cotask.Task(self.task_controller, priority=5, period=40, profile=True, trace=False)
        move = cotask.Task(self.task_move, priority=6, period=25, profile=True, trace=False)
        adjust_speed = cotask.Task(self.task_adjust_speed, priority=7, period=20, profile=True, trace=False)
        update_position = cotask.Task(self.task_update_position, priority=0, period=10, profile=True, trace=False)
        read_line = cotask.Task(self.task_read_line, priority=8, period = 30, profile=True, trace=False)
        bump = cotask.Task(self.task_bump, priority=8, period = 30, profile=True, trace=False)

        cotask.task_list.append(controller)
        cotask.task_list.append(read_line)
        cotask.task_list.append(move)
        cotask.task_list.append(adjust_speed)
        cotask.task_list.append(update_position)
        cotask.task_list.append(bump)
       
    ## Runs tasks until ctrl+c is pressed.
    def run_tasks(self):
        while True:
            try:
                cotask.task_list.pri_sched()
            except KeyboardInterrupt:
                self.motorL.set_duty(0)
                self.motorR.set_duty(0)
                break

    # TASK
    ## Main controller thread for the Romi.
    #
    # Controller is in charge of general phases. 
    # Romi has four phases: 
    # @arg @c normal line following
    # @arg @c  wall sequence
    # @arg @c end sequence
    # @arg @c stop
    def task_controller(self):
        while True:
            # general stop flag, romi wil stop if no other flag is set
            if not self.STOP:
                self.move_flag = True
                self.read_line_flag = True
            else:
                self.move_flag = False
                self.read_line_flag = False

            #sequence for moving arount the wall
            while self.WALL:
                #set initial positions
                pos = self.posAbs
                angle = abs(self.IMU.euler()[0])
                self.encR.update()
                posR = self.encR.get_position()

                #get next phase in sequence
                phase = self.WALL_PHASES.pop(0)

                #set flags to false
                self.read_line_flag = False
                self.sense_line_flag = False

                cond = True
                while cond:
                    self.move_flag = True
                    if phase == "back":
                        self.VELOCITY_RAD_L = -2 * self.SPEED
                        self.VELOCITY_RAD_R = -2 * self.SPEED
                        cond = self.posAbs > (pos - self.back_dist)
                    elif phase == "forward1":
                        self.VELOCITY_RAD_L = 2 * self.SPEED
                        self.VELOCITY_RAD_R = 2 * self.SPEED
                        cond = self.posAbs < (pos + self.forward1)
                    elif phase == "adjust forward":
                        self.VELOCITY_RAD_L = 2 * self.SPEED
                        self.VELOCITY_RAD_R = 2 * self.SPEED
                        cond = self.posAbs < (pos + self.adjust_forward_dist)
                    elif phase == "forward2":
                        self.VELOCITY_RAD_L = 1 * self.SPEED
                        self.VELOCITY_RAD_R = 1.5 * self.SPEED
                        self.read_line_flag = True
                        self.sense_line_flag = True
                        cond = not self.line_sensed_flag
                    elif phase == "turn45":
                        self.VELOCITY_RAD_L = 1 * self.SPEED
                        self.VELOCITY_RAD_R = -1 * self.SPEED
                        self.encR.update()
                        current_angle = self.encR.get_position()
                        cond =  (posR - current_angle < 375)
                    elif phase == "turn90":
                        self.VELOCITY_RAD_L = -1 * self.SPEED
                        self.VELOCITY_RAD_R = 1 * self.SPEED
                        self.encR.update()
                        current_angle = self.encR.get_position()
                        cond =  (current_angle - posR < self.angle)
                    elif phase == "orient":
                        self.VELOCITY_RAD_L = 1 * self.SPEED
                        self.VELOCITY_RAD_R = -1 * self.SPEED
                        current_angle = abs(self.IMU.euler()[0] )
                        cond = abs(abs(current_angle - angle)) < 45

                    yield

                #if there are no more phases the sequence is done
                if len(self.WALL_PHASES) == 0:
                    self.WALL = False
                    self.STOP = False
                    self.black_flag = False
                    #speed up for the home stretch
                    self.SPEED = 1.5 * self.SPEED
                yield

            #sequence for returning to the start
            while self.END:
                #set initial position
                angle = abs(self.IMU.euler()[0])
                pos = self.posAbs

                #reset count for tracking blank spaces
                self.white_goal = 3#9
                self.end_count = 0
                
                #get phase
                phase = self.END_PHASES.pop(0)

                cond = True
                while cond and phase != "stop":
                    self.move_flag = True
                    if phase == "forward":
                        self.VELOCITY_RAD_L = 2 * self.SPEED
                        self.VELOCITY_RAD_R = 2 * self.SPEED
                        cond = self.posAbs < (pos + self.return_dist)
                    elif phase == "adjust forward":
                        self.VELOCITY_RAD_L = 2 * self.SPEED
                        self.VELOCITY_RAD_R = 2 * self.SPEED
                        cond = self.posAbs < (pos + self.adjust_forward_dist)
                    elif phase == "turn180":
                        self.VELOCITY_RAD_L = -1 * self.SPEED
                        self.VELOCITY_RAD_R = 1 * self.SPEED
                        current_angle = abs(self.IMU.euler()[0] )
                        cond = not ((current_angle <= 3 and current_angle >= 0) or (current_angle <= 360 and current_angle >= 357) )#abs(abs(current_angle - angle)) < 165
                    elif phase == "line sense":
                        self.read_line_flag = True
                        cond = not (self.end_count >= self.white_goal)
                    yield

                #stop end sequence if no more phases
                if len(self.END_PHASES) == 0 or phase == "stop":
                    print("done with wall")
                    self.WALL = False
                    self.STOP = True
                    self.black_flag = False
                    self.END = False
                yield


            yield

    ## Task for polling the bump sensor.
    def task_bump(self):
        bmp = True
        while True:
            while bmp:
                #check to see if the bump sensor has been triggered
                if self.BMP.value() == 0 or self.BMP2.value() == 0:
                    self.STOP = True
                    self.WALL = True
                    bmp = False
                yield
            yield

    ## Task in charge of Romi movement.
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

            #if not moving stop the motors
            self.motorL.set_duty(0)
            self.motorR.set_duty(0)
            self.adjust_speed_flag = False
            self.update_position_flag = False
            yield

    ## Task in charge of adjusting individual wheel speeds. 
    def task_adjust_speed(self):
        # takes in desired velocity for both motors
        while True:
            while self.adjust_speed_flag:
                dutyR, dutyL = self.get_new_duty(self.VELOCITY_RAD_L, self.VELOCITY_RAD_R)  # this is what we would calculate with the IMU
                self.motorR.set_duty(dutyR)
                self.motorL.set_duty(dutyL)
                yield
            yield

    ## Task for updating the position of the Romi.
    def task_update_position(self):
        while True:
            # update romi encoders to track position
            while self.update_position_flag:
                self.encL.update()
                self.encR.update()
                self.posR = self.encR.get_position()
                self.posL = self.encL.get_position()
                self.posAbs = self.get_absolute_position()
                yield
            yield

    ## Task for reading the line sensor.
    def task_read_line(self):
        while True:
            #task for line sensing
            while self.read_line_flag:
                sensors = [self.SEN_0, self.SEN_2, self.SEN_3, self.SEN_4, self.SEN_5, self.SEN_7]
                vals = self.read(sensors)
                yield

            yield

    # FUNCTIONS

    ## Function for reading the line sensor.
    #
    # @param sensors Array of sensors on the line sensor.
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
        scaled[0] = -15*self.scale if sensorValues[5] > 900 else 0 #PIN0
        scaled[1] = -5*self.scale if sensorValues[4] > 900 else 0 #PIN2
        scaled[2] = -1*self.scale if sensorValues[3] > 700 else 0 #PIN3
        scaled[3] = 1*self.scale if sensorValues[2] > 700 else 0 #PIN4
        scaled[4] = 5*self.scale if sensorValues[1] > 900 else 0 #PIN5
        scaled[5] = 15*self.scale if sensorValues[0] > 900 else 0 #PIN7

        if not self.sense_line_flag:
            if scaled.count(0) == 6:
                if self.black_flag:
                    self.end_count += 1
                    print(f"line count: {self.end_count}")
                self.black_flag = False
            else:
                self.black_flag = True

            if self.end_count == self.white_goal:#22:
                self.STOP = True
                print("end triggered")
                self.END = True

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
        else:
            if scaled.count(0) != 6:
                print("line sensed")
                self.line_sensed_flag = True
        
    ## Funciton for calculating a new duty cycle for each motor base off the speed they are already going.
    #
    # @param vleft Desired left motor velocity
    # @param vright Desired right motor velocity
    def get_new_duty(self, vleft: int, vright: int):
        # function to calculate the new duty cycles of the motors
        velocityLreal = self.encL.get_velocity()
        velocityRreal = self.encR.get_velocity()

        L = 0
        R = 0
        Kpr = 3.8 #%s/rad
        Kpl = 3.06  # %s/rad

        #Feedback control for L Motor
        self.encL.update()
        ERRl = vleft - velocityLreal
        Fpl = Kpl*ERRl #%
        L = Fpl

        # Feedback control for R Motor
        self.encR.update()
        ERRr = vright - velocityRreal
        Fpr = Kpr * ERRr  # %
        R = Fpr 

        return R, L

    ## Function to get the absolute position of the Romi.
    def get_absolute_position(self):
        return (self.posL + self.posR) / 2

    ## Function to get the wheel velocity of the romi.
    #
    # @param times Seconds
    def get_wheel_velocity(self, times: int):
        rw = 2.76 #wheel radius, inches
        wromi = 5.55 #Chasi Diameter, inches
        D = 2*24 #circle diameter, inches

        C = D*math.pi #Circumfrance, inches
        Omegay = 2*math.pi/times #Overal rotational velocity, rad/s
        V = C/times #Centerline velocity, in/s
        vright = (V - Omegay*wromi)/ rw
        vleft = (2*V/rw) - vright

        return (vleft*2.4), (vright*2.4)

"""
FUNCTIONS
"""

## Initialize Romi motors using the Motor class
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


## Initialize Romi encoders using the Encoder class
def initialize_encoders():
    tim_R = Timer(1, period=65535, prescaler=0)
    enc_R = Encoder(tim_R, Pin.cpu.A8, Pin.cpu.A9, 1, 2)  # A8 and A9
    tim_L = Timer(3, period=65535, prescaler=0)
    enc_L = Encoder(tim_L, Pin.cpu.A7, Pin.cpu.A6, 1, 2)
    return enc_R, enc_L



if __name__ == '__main__':
    button = Pin(Pin.cpu.C13, Pin.IN, pull=Pin.PULL_UP)
    motorR, motorL = initialize_motors()
    encR, encL = initialize_encoders()
    i2c = machine.I2C(1)
    IMU = BNO055_2(i2c)

    tm = TaskManager(motorL, motorR, encL, encR, IMU)
    
    #press the button to start romi
    while button.value() == 1:
        continue
    tm.run_tasks()
