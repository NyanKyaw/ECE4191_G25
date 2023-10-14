# This file will represent our robot

import numpy as np
import RPi.GPIO as GPIO
import sys, os, time, math
import encoder
from encoder import Encoder

from gpiozero import RotaryEncoder, Robot, OutputDevice, PWMOutputDevice

class CustomRobot:
    def __init__(self, left_motor_pins, right_motor_pins, left_encder_pins=None, right_encoder_pins=None, distance_per_step=None):
        # Not running on remote machine, don't need ssh info -> Caveat might be CV, check with team

        # Do we need to define pins or hardcode? Defining prob better if we need to change :P
        self.left_motor_pins = left_motor_pins # PWM out
        self.right_motor_pins = right_motor_pins # PWM out

        #self.left_encoder_pins = kwargs['left_encoder_pints'] # Input to PI
        #self.right_encoder_pins = right_encoder_pins

        #self.left_encoder = RotaryEncoder(self.left_encoder_pins, max_steps=1000000)
        #self.right_encoder = RotaryEncoder(self.right_encoder_pins[0], self.right_encoder_pins[1],max_steps=1000000)
        #self.right_encoder = Encoder(self.right_encoder_pins[0], self.right_encoder_pins[1])

        # Motor
            # Set velocity (PWM)
                # Forward
                # Rotational
            # Receive feedback (odom)

        # Ultrasonic Sensor
            # Send trigger
            # Receive distance

        # Camera
            # Send trigger
            # Receive image
            # Process?

        # Model Params
            # Calibration
        self.distance_per_step = distance_per_step
        
        #self.robot = Robot(left=self.left_motor_pins, right=self.right_motor_pins, pwm=True)
        self.right_count = 0
        self.left_count = 0

    def drive_robot(self, velocity=[0,0], duration=None): 
        '''
        Function to set Robot's velocity.

        Inputs:
        - velocity: List, first element is forward velocity and second element is rotational velocity
        - time: Time in seconds to apply velocity
        '''
        start_time = time.time()
        time_interval = 0
        
        #init_left_steps = self.left_encoder.steps
        #init_right_steps = self.right_encoder.steps
        # while (time_interval) < duration:
        if velocity[0] > 0 and velocity[1] == 0: # Forward
            GPIO.output(self.left_motor_pins[0], GPIO.HIGH)
            GPIO.output(self.left_motor_pins[1], GPIO.LOW)
            GPIO.output(self.right_motor_pins[0], GPIO.LOW)
            GPIO.output(self.right_motor_pins[1], GPIO.HIGH)
        elif velocity[0] < 0 and velocity[1] == 0: # Backwards
            GPIO.output(self.left_motor_pins[0], GPIO.LOW)
            GPIO.output(self.left_motor_pins[1], GPIO.HIGH)
            GPIO.output(self.right_motor_pins[0], GPIO.HIGH)
            GPIO.output(self.right_motor_pins[1], GPIO.LOW)
        elif velocity[0] == 0 and velocity[1] > 0: # Right
            GPIO.output(self.left_motor_pins[0], GPIO.HIGH)
            GPIO.output(self.left_motor_pins[1], GPIO.LOW)
            GPIO.output(self.right_motor_pins[0], GPIO.HIGH)
            GPIO.output(self.right_motor_pins[1], GPIO.LOW)
        elif velocity[0] == 0 and velocity[1] < 0: # Left
            GPIO.output(self.left_motor_pins[0], GPIO.LOW)
            GPIO.output(self.left_motor_pins[1], GPIO.HIGH)
            GPIO.output(self.right_motor_pins[0], GPIO.LOW)
            GPIO.output(self.right_motor_pins[1], GPIO.HIGH)

            #if GPIO.input(self.right_encoder_pins[0]):
                #print("high")

            #time_interval = time.time() - start_time
            #print(self.right_encoder.read())
            #self.stop_drive()
            #left_steps = self.right_encoder.steps
            #right_steps = self.right_encoder.steps
            #left_speed, right_speed, left_distance, right_distance = self.compute_velocity(time_interval, left_steps=abs(left_steps-init_left_steps), right_steps=abs(right_steps-init_right_steps))
            #print(left_speed, right_speed)
	
        time.sleep(duration)
        self.stop_drive()
        #right_steps = self.right_encoder.steps
        #print(right_steps)
        #self.robot.stop()

    #def compute_velocity_distance(self, time_interval, left_steps, right_steps):
        #"""Compute the speed given the number of steps and the time interval."""
        # Assuming 1 step corresponds to 1 rotation -> isn't it 48 steps per rotation?
        #left_distance = left_steps * self.distance_per_step
        #right_distance = right_steps * self.distance_per_step
        #left_speed = left_distance / time_interval
        #right_speed = right_distance / time_interval
        #return left_speed, right_speed, left_distance, right_distance

        
    def stop_drive(self):
    	GPIO.output(self.left_motor_pins[1], GPIO.LOW)
    	GPIO.output(self.left_motor_pins[0], GPIO.LOW)
    	GPIO.output(self.right_motor_pins[1], GPIO.LOW)
    	GPIO.output(self.right_motor_pins[0], GPIO.LOW)
    	time.sleep(1)
    	
   







