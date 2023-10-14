import RPi.GPIO as GPIO
import utility.config as config
import utility.utility as utils
from scripts.main.robot import CustomRobot
import math
import numpy as np

def main():
    left_motor_pins, right_motor_pins, left_encoder_pins, right_encoder_pins = config.pinsetup()

    customRobot = CustomRobot(left_motor_pins, right_motor_pins, left_encoder_pins, right_encoder_pins)

