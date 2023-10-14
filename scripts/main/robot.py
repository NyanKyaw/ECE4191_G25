# This file will represent our robot

import numpy as np
import sys, os, time, math
import RPi.GPIO as GPIO
#from ultrasonic_sensor import ultrasonic_sensor

from gpiozero import RotaryEncoder, Robot, OutputDevice, PWMOutputDevice

class CustomRobot:
    def __init__(self, left_motor_pins, right_motor_pins, left_encoder_pins, right_encoder_pins):
        self.left_motor_pins = left_motor_pins 
        self.right_motor_pins = right_motor_pins

        self.left_encoder_pins = left_encoder_pins
        self.right_encoder_pins = right_encoder_pins
        
        self.x, self.y = [0,0]

    def drive_robot(self, left_wheel_dist, right_wheel_dist, duration=None):
        '''
        Function to set Robot's velocity.

        Inputs:
        - velocity: List, first element is forward velocity and second element is rotational velocity
            - Values range from -1 to 1, with negative values corresponding to reverse movement
            - Magnitude of value controls duty cycle (speed)
        - time: Time in seconds to apply velocity

        ** Note we are not allowing for arc movements, only linear movement and rotation **
        '''

        # is one step one revolution of the motor shaft/gearbox output, or is one step one count of the motor shaft/gearbox output?
        # Gonna assume its the later for now, assuming that its motor shaft as well not gearbox (1 step = 1/48 revolution)
        start_time = time.time()
        time_interval = 0
        init_left_steps = self.left_encoder.steps
        init_right_steps = self.right_encoder.steps
        sensor1, sensor2, sensor3 = 0, 0, 0
        while self.left_encoder.steps < np.abs(left_wheel_dist) and not (sensor1 or sensor2 or sensor3): # both wheels should always be achieving same distance
            #sensor1, sensor2, sensor3 = ultrasonic_sensor()
            if left_wheel_dist and right_wheel_dist> 0: # Forward
                GPIO.output(self.left_motor_pins[0], 1)
                GPIO.output(self.left_motor_pins[1], 0)
                GPIO.output(self.right_motor_pins[0], 0)
                GPIO.output(self.right_motor_pins[1], 1)
            elif left_wheel_dist and right_wheel_dist< 0: # Backwards
                GPIO.output(self.left_motor_pins[0], 0)
                GPIO.output(self.left_motor_pins[1], 1)
                GPIO.output(self.right_motor_pins[0], 1)
                GPIO.output(self.right_motor_pins[1], 0)
            elif left_wheel_dist >0 and right_wheel_dist< 0:# Right
                GPIO.output(self.left_motor_pins[0], 1)
                GPIO.output(self.left_motor_pins[1], 0)
                GPIO.output(self.right_motor_pins[0], 1)
                GPIO.output(self.right_motor_pins[1], 0)
            elif left_wheel_dist <0 and right_wheel_dist> 0: # Left
                GPIO.output(self.left_motor_pins[0], 0)
                GPIO.output(self.left_motor_pins[1], 1)
                GPIO.output(self.right_motor_pins[0], 0)
                GPIO.output(self.right_motor_pins[1], 1)
            else:
                print("Incorrect wheel distance inputted\n\n Left Wheel Dist: " + left_wheel_dist + "Right Wheel Dist: "+ right_wheel_dist + "\n")

            time_interval = time.time() - start_time
            left_steps = self.right_encoder.steps
            right_steps = self.right_encoder.steps


           # left_speed, right_speed, left_distance, right_distance = self.compute_velocity_distance(time_interval, left_steps=abs(left_steps-init_left_steps), right_steps=abs(right_steps-init_right_steps))

            # From these left_distance, right_distance, we need to find a way to calc angle/heading and from there we can estimate global x,y
            # Maybe heading can be derived from left/right speed

        self.stop_drive()

    def compute_velocity_distance(self, time_interval, left_steps, right_steps):
        """Compute the speed given the number of steps and the time interval."""
        # Assuming 1 step corresponds to 1/48 revolution 
        left_distance = left_steps * (self.distance_per_revolution/48)
        right_distance = right_steps * (self.distance_per_revolution/48)
        left_speed = left_distance / time_interval
        right_speed = right_distance / time_interval
        return left_speed, right_speed, left_distance, right_distance

    def stop_drive(self):
        GPIO.output(self.left_motor_pins[0], 0)
        GPIO.output(self.left_motor_pins[1], 0)
        GPIO.output(self.right_motor_pins[0], 0)
        GPIO.output(self.right_motor_pins[1], 0)

        self.left_motor_pins[2].ChangeDutyCycle(100)
        self.right_motor_pins[2].ChangeDutyCycle(100)





