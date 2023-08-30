### We will implement all control scripts in this file, and it will be what is run ###

import RPi.GPIO as GPIO
import config
from robot import CustomRobot

GPIO.setmode(GPIO.BCM)

### Set MOTOR Pins ###
left_motor_pins = (config.ML_PWM1, config.ML_PWM2)
right_motor_pins = (config.MR_PWM1, config.MR_PWM2)

left_encoder_pins = (config.ML_ENCA, config.ML_ENCB)
right_encoder_pins = (config.MR_ENCA, config.MR_ENCB)

### Robot Params ###
distance_per_step = 0.1 * 3.14159  # Wheel circumference = diameter * pi

def main():
    customRobot = CustomRobot(left_motor_pins, right_motor_pins, left_encoder_pins, right_encoder_pins, distance_per_step)

    while True:
        customRobot.drive_robot(velocity=[1,0], duration=2)
        customRobot.drive_robot(velocity=[-1,0], duration=2)
        customRobot.drive_robot(velocity=[0,1], duration=2)
        customRobot.drive_robot(velocity=[0,-1], duration=2)

    GPIO.cleanup()
    
