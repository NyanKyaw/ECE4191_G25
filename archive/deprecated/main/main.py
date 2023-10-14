### We will implement all control scripts in this file, and it will be what is run ###

import RPi.GPIO as GPIO
import config
from robot import CustomRobot
import numpy as np
import time

GPIO.setmode(GPIO.BCM)

### Set MOTOR Pins ###
#left_motor_pins = (config.ML_PWM1, config.ML_PWM2)
left_motor_pins = (config.Left_IN1, config.Left_IN2, config.Left_PWM)
#right_motor_pins = (config.MR_PWM1, config.MR_PWM2)
right_motor_pins = (config.Right_IN3, config.Right_IN4, config.Right_PWM)

left_encoder_pins = (config.Left_ENCA, config.Left_ENCB)
right_encoder_pins = (config.Right_ENCA, config.Right_ENCB)

### Robot Params ###
distance_per_step = 0.1 * 3.14159  # Wheel circumference = diameter * pi
wheel_dis_to_centre = 11.3

# Setup pins
GPIO.setup(config.Left_IN1, GPIO.OUT)
GPIO.setup(config.Left_IN2, GPIO.OUT)
GPIO.setup(config.Right_IN3, GPIO.OUT)
GPIO.setup(config.Right_IN4, GPIO.OUT)
GPIO.setup(config.Left_PWM, GPIO.OUT)
GPIO.setup(config.Right_PWM, GPIO.OUT)
GPIO.setup(config.Right_ENCA, GPIO.IN)
GPIO.setup(config.Right_ENCB, GPIO.IN)

# GPIO.setup(config.Right_ENCB, GPIO.OUT)
# GPIO.output(config.Right_ENCB, GPIO.HIGH)
# time.sleep(10)

# rightA_count = 0
# rightB_count=0

# def rightA_rise():
#     rightA_count += 1
#     print(rightA_count)

# def rightB_rise():
#     rightB_count += 1
#     print(rightB_count)

# # Add event detectors
# GPIO.add_event_detect(config.Right_ENCA, GPIO.RISING, callback=rightA_rise, bouncetime=10)
# GPIO.add_event_detect(config.Right_ENCB, GPIO.RISING, callback=rightB_rise, bouncetime=10)

# Set up the PWM frequencies
pwm1 = GPIO.PWM(config.Left_PWM, 1000)
pwm2 = GPIO.PWM(config.Right_PWM, 1000)



def drive(LW_dist, RW_dist, customRobot):
    speed =15
    distance_for_10_input = 3
    scaling_factor = 10/distance_for_10_input
   # print("Scaling factor")
    #print(scaling_factor)
    LW_scaled_dist = scaling_factor*LW_dist
   # print("LW_scaled_dist")
    #print(LW_scaled_dist)
    RW_scaled_dist = scaling_factor*RW_dist
    duration_needed = np.abs(LW_dist)/speed
    # print("LW Distance: ")
    # print(LW_dist)
    # print("Duration: ")
    # print(duration_needed)
    #print("Steps:")
    #print(customRobot.right_encoder.steps)

    if (LW_dist == RW_dist and LW_dist>0): # FORWARD
        #print("starting forward move")
        customRobot.drive_robot(velocity=[-1,0], duration=duration_needed)
        print("forward")
    elif (LW_dist>RW_dist): #   RIGHT
        print("starting right move")
        customRobot.drive_robot(velocity=[0,-1], duration=duration_needed)
        print("right ended")
    elif (LW_dist<RW_dist): #LEFT
        customRobot.drive_robot(velocity=[0,1], duration=duration_needed)
        print("left")
    elif (LW_dist == RW_dist and LW_dist<0): # BACKWARD
        customRobot.drive_robot(velocity=[1,0], duration=duration_needed)
        print("backward")
    
    #print(customRobot.right_encoder.steps)

def dis_from_angle(angle): # angle always in deg
  print("Distance from Angle: ")
  print((angle*np.pi*wheel_dis_to_centre)/180)
  return (angle*np.pi*wheel_dis_to_centre)/180

def trans_func(next_x, next_y, cur_x, cur_y, customRobot):
  distance = np.sqrt(np.square(next_x - cur_x) + np.square(next_y - cur_y))
  drive(distance, distance, customRobot)

def angle_finder(next_x, next_y, cur_angle, cur_x, cur_y):
  opp = next_x - cur_x
  print("opp")
  print(opp)
  adj = next_y - cur_y
  print("adj")
  print(adj)

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
    angle_N_point = np.arctan(opp/adj)*180/np.pi

    print("Angle N Point")
    print(angle_N_point)


  angle_total = angle_N_point-cur_angle

  print("Angle Total")
  print(angle_total)

  return (angle_total%360) #want it output between 0 and 360 as that is what rot func takes in

def rot_func(angle, customRobot):

  if (angle >= 0 and angle <= 180):
    LW_dist = dis_from_angle(angle) # Left Wheel
    RW_dist = -1 * dis_from_angle(angle) # Right Wheel
  else:
    LW_dist = -1 * dis_from_angle(360 - angle)
    RW_dist = dis_from_angle(360 - angle)
    print("Dist: ")
    print(LW_dist)

  drive(LW_dist, RW_dist,customRobot)

def encoder_callback(channel):
  print("INPUTED SIGNAL")

# def detect_pwm(pin):
#    try:
#       while True:
#          GPIO.wait_for_edge(pin, GPIO.RISING)

def main():
    customRobot = CustomRobot(left_motor_pins, right_motor_pins, distance_per_step=distance_per_step, left_encoder_pins=left_encoder_pins, right_encoder_pins=right_encoder_pins)
    
    pwm1.start(100)
    pwm2.start(100)

    #GPIO.add_event_detect(16, GPIO.BOTH, callback=encoder_callback)

    # x_green = 30 #cm
    # y_green = 20
    # angle_green = 0

    # x_red = 90
    # y_red = 80

    # x_yel = 30
    # y_yel = 80

    # print("TESTING")

    # # GREEN -> RED
    # customRobot.drive_robot(velocity=[1,0], duration=5)
    # #drive(40,40, customRobot)
    # print("TEST ENDED")
    # angle_red = angle_finder(x_red, y_red, angle_green, x_green, y_green)
    # print("Angle Red: ")
    # print(angle_red)
    # rot_func(angle_red, customRobot)
    # trans_func(x_red, y_red, x_green, y_green, customRobot)
    
    try:
    	while True:
            """# This is calibration code
            turning_dist = ((180*np.pi*11)/180)*0.9
            print("Turning dist")
            print(turning_dist)
            RW_dist = -1*turning_dist
            drive(-1*RW_dist, -1*turning_dist, customRobot)
            """
            #This is path projection code
            angle = (np.arctan(50/60)*180)/np.pi
            print("Angle")
            print(angle) # 39.806
            one_rotate = ((angle*np.pi*11)/180)*0.9
            drive(one_rotate, -1*one_rotate, customRobot)
            break
            
            two_translate = np.sqrt(50*50 + 60*60)
            drive(two_translate, two_translate, customRobot)
            
            two_rotate = 90 + angle
            drive(two_rotate*-1, two_rotate, customRobot)
            
            three_translate = 50
            drive(three_translate, three_translate, customRobot)
            
            
            
            #drive(-25,-25,customRobot)
            # print("right")
            # drive(80,-80, customRobot)
            # print("left")
            # drive(-20,20,customRobot)
            # print("backwards")
            # drive(40,-40, customRobot)
            # print("forward")
            # #print("\nSteps: " + customRobot.left_encoder.steps)
            # customRobot.drive_robot(velocity=[1,0], duration=5)
            # #print("\nSteps: " + customRobot.left_encoder.steps)
            # print("backward")
            # customRobot.drive_robot(velocity=[-1,0], duration=5)
            # print("right")
            # customRobot.drive_robot(velocity=[0,1], duration=5)
            # print("left")
            # customRobot.drive_robot(velocity=[0,-1], duration=5)
            
    except KeyboardInterrupt:
        pwm1.stop()
        pwm2.stop()
        GPIO.cleanup()
    	

    
if __name__ == "__main__":
    main()
