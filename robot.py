# This file will represent our robot

import numpy as np
import sys, os, time, math

from gpiozero import RotaryEncoder, Robot, OutputDevice, PWMOutputDevice

class CustomRobot:
    def __init__(self, **kwargs):
        # Not running on remote machine, don't need ssh info -> Caveat might be CV, check with team

        # Do we need to define pins or hardcode? Defining prob better if we need to change :P
        self.left_motor_pins = kwargs['left_motor_pins'] # PWM out
        self.right_motor_pins = kwargs['right_motor_pins'] # PWM out

        self.left_encoder_pins = kwargs['left_encoder_pints'] # Input to PI
        self.right_encoder_pins = kwargs['right_encoder_pins'] # Input to PI

        self.left_encoder = RotaryEncoder(self.left_encoder_pins, max_steps=1000000)
        self.right_encoder = RotaryEncoder(self.right_encoder_pins, max_steps=1000000)

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
        self.distance_per_step = kwargs['distance_per_step']
        
        self.robot = Robot(left=self.left_motor_pins, right=self.right_motor_pins, pwm=True)

    def drive_robot(self, velocity=[0,0], duration=None): 
        '''
        Function to set Robot's velocity.

        Inputs:
        - velocity: List, first element is forward velocity and second element is rotational velocity
        - time: Time in seconds to apply velocity
        '''
        start_time = time.time()
        time_interval = 0
        init_left_steps = self.left_encoder.steps
        init_right_steps = self.right_encoder.steps
        while (time_interval) < duration:
            if velocity[0] > 0 and velocity[1] == 0: # Forward
                self.robot.forward()
            elif velocity[0] < 0 and velocity[1] == 0: # Backwards
                self.robot.backwards()
            elif velocity[0] == 0 and velocity[1] > 0: # Right
                self.robot.right()
            elif velocity[0] == 0 and velocity[1] < 0: # Left
                self.robot.left()
            
            time_interval = time.time() - start_time
            left_steps = self.right_encoder.steps
            right_steps = self.right_encoder.steps
            left_speed, right_speed, left_distance, right_distance = self.compute_velocity(time_interval, left_steps=abs(left_steps-init_left_steps), right_steps=abs(right_steps-init_right_steps))
            print(left_speed, right_speed)

        self.robot.stop()

    def compute_velocity_distance(self, time_interval, left_steps, right_steps):
        """Compute the speed given the number of steps and the time interval."""
        # Assuming 1 step corresponds to 1 rotation -> isn't it 48 steps per rotation?
        left_distance = left_steps * self.distance_per_step
        right_distance = right_steps * self.distance_per_step
        left_speed = left_distance / time_interval
        right_speed = right_distance / time_interval
        return left_speed, right_speed, left_distance, right_distance







