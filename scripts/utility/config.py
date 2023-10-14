### Import Packages ###
import RPi.GPIO as GPIO
import config
from scripts.main.robot import CustomRobot
import math
import numpy as np
from enum import Enum

class MotorPins(Enum):
    LEFT_IN1 = 17,
    LEFT_IN2 = 27,
    LEFT_PWM = 22,
    LEFT_ENCA = 5,
    LEFT_ENCB = 6,
    RIGHT_IN3 = 24,
    RIGHT_IN4 = 25,
    RIGHT_PWM = 23,
    RIGHT_ENCA = 15,
    RIGHT_ENCB = 26

### IMPORTANT PARAMETERS ###
class RobotParams(Enum):
    WHEEL_DIAMETER = 56,
    WHEEL_DISTANCE_TO_CENTRE = 11.5,
    COUNT_PER_REV = 48, 
    DISTANCE_PER_REVOLUTION = WHEEL_DIAMETER * np.pi # circumference

def pinsetup(pwm_frequency = 100):
    ### Setup In/Out/PWM Pins ###
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MotorPins.LEFT_IN1.value, GPIO.OUT)
    GPIO.setup(MotorPins.LEFT_IN2.value, GPIO.OUT)
    GPIO.setup(MotorPins.RIGHT_IN3.value, GPIO.OUT)
    GPIO.setup(MotorPins.RIGHT_IN4.value, GPIO.OUT)
    GPIO.setup(MotorPins.LEFT_PWM.value, GPIO.OUT)
    GPIO.setup(MotorPins.RIGHT_PWM.value, GPIO.OUT)

    GPIO.setup(config.Left_ENCA, GPIO.IN)
    GPIO.setup(config.Left_ENCB, GPIO.IN)
    GPIO.setup(config.Right_ENCA, GPIO.IN)
    GPIO.setup(config.Right_ENCB, GPIO.IN)

    ### Set MOTOR Pins ###
    left_motor_pins = (config.Left_IN1, config.Left_IN2, config.Left_PWM)
    right_motor_pins = (config.Right_IN3, config.Right_IN4, config.Right_PWM)

    left_encoder_pins = (config.Left_ENCA, config.Left_ENCB)
    right_encoder_pins = (config.Right_ENCA, config.Right_ENCB)

    pwm_left = GPIO.PWM(config.Left_PWM, pwm_frequency) 
    pwm_right = GPIO.PWM(config.Right_PWM, pwm_frequency)

    return left_motor_pins, right_motor_pins, left_encoder_pins, right_encoder_pins, pwm_left, pwm_right