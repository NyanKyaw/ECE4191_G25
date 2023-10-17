### Import Packages ###
import RPi.GPIO as GPIO
import config
import math
import numpy as np
from enum import Enum
import cv2
# from picamera.array import PiRGBArray
# from picamera import PiCamera
import time

class MotorPins(Enum):
    LEFT_IN1 = 17
    LEFT_IN2 = 27
    LEFT_PWM = 22
    LEFT_ENCA = 5
    LEFT_ENCB = 6
    RIGHT_IN3 = 24
    RIGHT_IN4 = 25
    RIGHT_PWM = 23
    RIGHT_ENCA = 15
    RIGHT_ENCB = 26

class Goals(Enum): # tune
    BIN_A = (0, 1)
    BIN_B = (0.5, 1)
    BIN_C = (1, 1)

### IMPORTANT PARAMETERS ###
class RobotParams(Enum):
    WHEEL_DIAMETER = 56
    WHEEL_DISTANCE_TO_CENTRE = 11.5
    COUNT_PER_REV = 48
    DISTANCE_PER_REVOLUTION = 56 * np.pi # circumference
    LW_VELOCITY = 0.625
    RW_VELOCITY = 0.625

def pinsetup(pwm_frequency = 100):
    ### Setup In/Out/PWM Pins ###
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MotorPins.LEFT_IN1.value, GPIO.OUT)
    GPIO.setup(MotorPins.LEFT_IN2.value, GPIO.OUT)
    GPIO.setup(MotorPins.RIGHT_IN3.value, GPIO.OUT)
    GPIO.setup(MotorPins.RIGHT_IN4.value, GPIO.OUT)
    GPIO.setup(MotorPins.LEFT_PWM.value, GPIO.OUT)
    GPIO.setup(MotorPins.RIGHT_PWM.value, GPIO.OUT)

    GPIO.setup(MotorPins.LEFT_ENCA.value, GPIO.IN)
    GPIO.setup(MotorPins.LEFT_ENCB.value, GPIO.IN)
    GPIO.setup(MotorPins.RIGHT_ENCA.value, GPIO.IN)
    GPIO.setup(MotorPins.RIGHT_ENCB.value, GPIO.IN)

    ### Set MOTOR Pins ###
    left_motor_pins = (MotorPins.LEFT_IN1.value, MotorPins.LEFT_IN2.value, MotorPins.LEFT_PWM.value)
    right_motor_pins = (MotorPins.RIGHT_IN3.value, MotorPins.RIGHT_IN4.value, MotorPins.RIGHT_PWM.value)

    left_encoder_pins = (MotorPins.LEFT_ENCA, MotorPins.LEFT_ENCB)
    right_encoder_pins = (MotorPins.RIGHT_ENCA.value, MotorPins.RIGHT_ENCB.value)

    print(MotorPins.LEFT_IN1.value)

    pwm_left = GPIO.PWM(MotorPins.LEFT_PWM.value, pwm_frequency) 
    pwm_right = GPIO.PWM(MotorPins.RIGHT_PWM.value, pwm_frequency)

    return left_motor_pins, right_motor_pins, left_encoder_pins, right_encoder_pins, pwm_left, pwm_right

def camerasetup():
    # camera = PiCamera()
    # camera.resolution = (1920, 1080)  # Set resolution of the camera
    # camera.framerate = 32
    # rawCapture = PiRGBArray(camera, size=(1920, 1080))

    # time.sleep(0.1)  # Allow the camera to warm up
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) #set resolution of camera
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720) #resolution of camera

    detector = cv2.QRCodeDetector()

    return detector, cap