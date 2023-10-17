# This file will represent our robot

import numpy as np
import sys, os, time, math
import RPi.GPIO as GPIO
#from ultrasonic_sensor import ultrasonic_sensor
import utility as utils
import servoTest as servoTest

from gpiozero import RotaryEncoder, Robot, OutputDevice, PWMOutputDevice

class CustomRobot:
    def __init__(self, left_motor_pins, right_motor_pins, left_encoder_pins, right_encoder_pins, wheel_velocities):
        self.left_motor_pins = left_motor_pins 
        self.right_motor_pins = right_motor_pins

        self.left_encoder_pins = left_encoder_pins
        self.right_encoder_pins = right_encoder_pins
        
        self.x, self.y = 0,0

        self.lw_velocity, self.rw_velocity = wheel_velocities

        self.left, self.right = 90, -90 # obv put these in enums or something later

    def drive_to_goal(self, goal):
        delta_x = goal[0] - self.x # change in x coord
        delta_y = goal[1] - self.y # change in y coord

        # always move forward first, so use delta_y to calculate duration
        trans_duration_y = delta_y / self.lw_velocity
        self.drive_robot(1, -1, trans_duration_y)

        # then rotate if needed
        self.rotate(angle=self.right)

        # then translate if needed
        trans_duration_x = delta_x / self.lw_velocity
        self.drive_robot(1, -1, trans_duration_x)

        # then rotate to face bin
        self.rotate(angle=self.left)

        # NOTE: LIMIT SWITCHING

        ### Note could hardcode it so if binA, no rotation, but being safe here in case our x has drifted
        return trans_duration_x, trans_duration_y
    
    def drive_to_origin(self, x_duration, y_duration):
        self.rotate(angle=self.left)
        self.drive_robot(1, -1, x_duration)
        self.rotate(angle=self.left)
        self.drive_robot(1, -1, y_duration)

    def rotate(self, angle): 
        # Define anti-clockwise as positive
        rot_duration = utils.dis_from_angle(angle) / self.lw_velocity

        if (angle >= 0 and angle <= 180): # rotate left
            self.drive_robot(-1, -1, rot_duration) # based off calibrating robot velocity gsheets
        else:
            self.drive_robot(1, 1, rot_duration) # based off calibrating robot velocity gsheets

    # PROCESS 1, will run as long as sensor event not triggered, will also use sensor information to update x, y for now
    def drive_robot(self, left_wheel_dir, right_wheel_dir, duration=None):
        '''
        Function to set Robot's velocity.

        Inputs:
        - velocity: List, first element is forward velocity and second element is rotational velocity
            - Values range from -1 to 1, with negative values corresponding to reverse movement
            - Magnitude of value controls duty cycle (speed) -> P Control!
        - time: Time in seconds to apply velocity

        ** Note we are not allowing for arc movements, only linear movement and rotation **
        '''

        start_time = time.time()
        dt = None
        while time.time() - start_time < duration:
            if left_wheel_dir and right_wheel_dir > 0: # Forward
                GPIO.output(self.left_motor_pins[0], 1)
                GPIO.output(self.left_motor_pins[1], 0)
                GPIO.output(self.right_motor_pins[0], 0)
                GPIO.output(self.right_motor_pins[1], 1)
            elif left_wheel_dir and right_wheel_dir < 0: # Backwards
                GPIO.output(self.left_motor_pins[0], 0)
                GPIO.output(self.left_motor_pins[1], 1)
                GPIO.output(self.right_motor_pins[0], 1)
                GPIO.output(self.right_motor_pins[1], 0)
            elif left_wheel_dir > 0 and right_wheel_dir < 0:# Right
                GPIO.output(self.left_motor_pins[0], 1)
                GPIO.output(self.left_motor_pins[1], 0)
                GPIO.output(self.right_motor_pins[0], 1)
                GPIO.output(self.right_motor_pins[1], 0)
            elif left_wheel_dir < 0 and right_wheel_dir > 0: # Left
                GPIO.output(self.left_motor_pins[0], 0)
                GPIO.output(self.left_motor_pins[1], 1)
                GPIO.output(self.right_motor_pins[0], 0)
                GPIO.output(self.right_motor_pins[1], 1)
            if dt is None:
                dt = time.time() - start_time
            self.x = self.x + dt*self.lw_velocity # rough estiamte of x,y position
            self.y = self.y + dt*self.rw_velocity

        self.stop_drive()

    def stop_drive(self):
        GPIO.output(self.left_motor_pins[0], 0)
        GPIO.output(self.left_motor_pins[1], 0)
        GPIO.output(self.right_motor_pins[0], 0)
        GPIO.output(self.right_motor_pins[1], 0)

    # Process 2: Will read ultrasonics and get values, update the values stored by robot. Can also raise an event to stop drive, do obstacle avoidance code and return
    def read_ultrasonics(self):
        pass

    def deploy_packages(self):
        servoTest.test_servo()
    
    def reset_x_y(self):
        self.x, self.y = 0,0







