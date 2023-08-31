### We will implement all control scripts in this file, and it will be what is run ###

import RPi.GPIO as GPIO
import config
from robot import CustomRobot
import math

### Setup In/Out/PWM Pins ###
GPIO.setmode(GPIO.BCM)
GPIO.setup(config.Left_IN1, GPIO.OUT)
GPIO.setup(config.Left_IN2, GPIO.OUT)
GPIO.setup(config.Right_IN3, GPIO.OUT)
GPIO.setup(config.Right_IN4, GPIO.OUT)
GPIO.setup(config.Left_PWM, GPIO.OUT)
GPIO.setup(config.Right_PWM, GPIO.OUT)

pwm_left = GPIO.PWM(config.Left_PWM, 100) # this is the frequency, can increase frequency to top speed
pwm_right = GPIO.PWM(config.Right_PWM, 100)

GPIO.setup(config.Left_ENCA, GPIO.IN)
GPIO.setup(config.Left_ENCB, GPIO.IN)
GPIO.setup(config.Right_ENCA, GPIO.IN)
GPIO.setup(config.Right_ENCB, GPIO.IN)

### Set MOTOR Pins ###
left_motor_pins = (config.Left_IN1, config.Left_IN2, config.Left_PWM)
right_motor_pins = (config.Right_IN3, config.Right_IN4, config.Right_PWM)

left_encoder_pins = (config.Left_ENCA, config.Left_ENCB)
right_encoder_pins = (config.Right_ENCA, config.Right_ENCB)

### Robot Params ###
wheel_diameter = # unknown -> either measure or calibrate

### Important Constants ###
distance_per_revolution = wheel_diameter * math.pi # Wheel circumference = diameter * pi

def main():
    customRobot = CustomRobot(left_motor_pins, right_motor_pins, left_encoder_pins, right_encoder_pins, distance_per_revolution)

    try:
        pwm_left.start(100) # this is the duty cycle, not frequency -> change duty cycle to change speed
        pwm_right.start(100)
        while True:
            customRobot.drive_robot(velocity=[1,0], duration=2)
            customRobot.drive_robot(velocity=[-1,0], duration=2)
            customRobot.drive_robot(velocity=[0,1], duration=2)
            customRobot.drive_robot(velocity=[0,-1], duration=2)
    except KeyboardInterrupt:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
    
