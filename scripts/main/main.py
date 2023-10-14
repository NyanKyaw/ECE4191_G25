### We will implement all control scripts in this file, and it will be what is run ###

import RPi.GPIO as GPIO
import config
from scripts.main.robot import CustomRobot
import math
import numpy as np

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
wheel_diameter = 56
wheel_dis_to_centre = 11.5 # rough estimate
count_per_rev = 48

### Important Constants ###
distance_per_revolution = wheel_diameter * math.pi # Wheel circumference = diameter * pi


def dis_from_angle(angle): # angle always in deg
  return (angle*np.pi*wheel_dis_to_centre)/180

def angle_from_dis(distance):
  return (distance*180)/(np.pi*wheel_dis_to_centre)

def dis_from_count(count):
  return (count*distance_per_revolution)/count_per_rev

def trans_func(next_x, next_y, cur_x, cur_y, customRobot):
  distance = np.sqrt(np.square(next_x - cur_x) + np.square(next_y - cur_y))
  customRobot.drive_robot(distance, distance, duration=None)

def angle_finder(next_x, next_y, cur_angle, cur_x, cur_y):
  opp = next_x - cur_x
  adj = next_y - cur_y

  if opp==0:
    if adj>0:
      angle_N_point = 0
    elif adj<0:
      angle_N_point = 0
  elif adj==0:
    if opp>0:
      angle_N_point = 90
    elif opp<0:
      angle_N_point = 270
  else:
    angle_N_point = np.arctan(opp/adj)

  angle_total = angle_N_point-cur_angle

  return (angle_total%360) #want it output between 0 and 360 as that is what rot func takes in

def rot_func(angle, customRobot):

  if (angle >= 0 and angle <= 180):
    LW_dist = dis_from_angle(angle) # Left Wheel
    RW_dist = -1 * dis_from_angle(angle) # Right Wheel
  else:
    LW_dist = -1 * dis_from_angle(360 - angle)
    RW_dist = dis_from_angle(360 - angle)

  customRobot.drive_robot(LW_dist, RW_dist)

# Obstacle detection - Simple

def angle_obstacle_finder(sensor1, sensor2, sensor3, sensor4, sensor5):
  if (sensor3): # front sensor
    angle_1 = 270
    angle_2 = 90
  elif (sensor4): # right angled sensor
    angle_1 = 315
    angle_2 = 90
  elif (sensor2 and not sensor3): # left angled sensor
    angle_1 = 45
    angle_2 = 270

  return angle_1, angle_2

def obst_detect(sensor1, sensor2, sensor3, sensor4, sensor5, initial_x, initial_y, customRobot):
  #save x and y coordinate -
  count_x =0
  count_y =0

  # maybe stabilise this as sensor 3 will go high then low during the course of the function
  angle_1, angle_2 = angle_obstacle_finder(sensor1, sensor2, sensor3, sensor4, sensor5)

  # ROTATE 90 DEG - 1
  rot_func(angle_1)

  # MOVE ALONG OBSTACLE IN Y AXIS
  LW = 1
  RW = 1
  while (sensor5):
    count_y +=1
    customRobot.drive_robot(LW, RW)# just running this function again and again until sensor 5 no longer detects
  # Distance of 15cm (robot centre to back)
  LW = 15
  RW = 15
  customRobot.drive_robot(LW, RW)

  # ROTATE 90 DEG - 2
  rot_func(angle_2)
  # MOVE ALONG OBSTACLE IN Y AXIS
  LW = 1
  RW = 1
  while (sensor5):
    count_x +=1
    customRobot.drive_robot(LW, RW)
  # Distance of 15cm (robot centre to back)
  LW = 13.5
  RW = 13.5
  customRobot.drive_robot(LW, RW)

  # RECALCULATE ROUTE - use the location from the computer vision (or have to use odometry)
  x_cur = initial_x + count_x
  y_cur = initial_y + count_y
  cur_angle = 270 + angle_1 + angle_2 - 360 # 360 hardcoded - can change to /360, remainder * 360
  x_yel = 30
  y_yel = 80

  angle = angle_finder(x_yel, y_yel, cur_angle, x_cur, y_cur)
  rot_func(angle, customRobot)
  # Translate
  trans_func(x_yel, y_yel, x_cur, y_cur, customRobot)
  # Things to think about, if the robot bumps into the obstacle twice (Most likely due to not turning properly - may need to implement the right angle checker)


def main():
    customRobot = CustomRobot(left_motor_pins, right_motor_pins, left_encoder_pins, right_encoder_pins, distance_per_revolution)

    x_green = 30 #cm
    y_green = 20
    angle_green = 0

    x_red = 90
    y_red = 80

    x_yel = 30
    y_yel = 80

    # GREEN -> RED
    angle_red = angle_finder(x_red, y_red, angle_green, x_green, y_green)
    rot_func(angle_red, customRobot)
    trans_func(x_red, y_red, x_green, y_green, customRobot)

    #turn LED light on or stop for 3 second

    # RED -> YELLOW
    angle = angle_finder(x_yel, y_yel, angle_red, x_red, y_red)
    rot_func(angle, customRobot)
    trans_func(x_yel, y_yel, x_red, y_red, customRobot)
    
    # try:
    #     pwm_left.start(100) # this is the duty cycle, not frequency -> change duty cycle to change speed
    #     pwm_right.start(100)
    #     while True:
    #         customRobot.drive_robot(velocity=[1,0], duration=2)
    #         customRobot.drive_robot(velocity=[-1,0], duration=2)
    #         customRobot.drive_robot(velocity=[0,1], duration=2)
    #         customRobot.drive_robot(velocity=[0,-1], duration=2)

            

    # except KeyboardInterrupt:
    #     pwm_left.stop()
    #     pwm_right.stop()
    #     GPIO.cleanup()
     