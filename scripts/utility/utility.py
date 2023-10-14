import utility.config as config
import numpy as np

### Important Constants ###
wheel_dis_to_centre = config.RobotParams.WHEEL_DISTANCE_TO_CENTRE.value
count_per_rev = config.RobotParams.COUNT_PER_REV.value
distance_per_revolution = config.RobotParams.DISTANCE_PER_REVOLUTION.value

def dis_from_angle(angle):
  return (angle*np.pi*wheel_dis_to_centre)/180 # Return linear distance from angle

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
