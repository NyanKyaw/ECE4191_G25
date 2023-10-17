import RPi.GPIO as GPIO
import config as config
import utility as utils
import parceldetector as parceldetector
from robot import CustomRobot
import math
import numpy as np
import multiprocessing

def main():
  left_motor_pins, right_motor_pins, left_encoder_pins, right_encoder_pins, pwm_left, pwm_right = config.pinsetup()

  detector, camera = config.camerasetup()

  customRobot = CustomRobot(left_motor_pins, right_motor_pins, left_encoder_pins, right_encoder_pins, wheel_velocities = (config.RobotParams.LW_VELOCITY.value * config.RobotParams.DISTANCE_PER_REVOLUTION.value, config.RobotParams.RW_VELOCITY.value * config.RobotParams.DISTANCE_PER_REVOLUTION.value))

  goal = None

  while True: # infinite loop

    # 1. check camera for QR code detection
      # if not detected detected, continue
      # if detected, return bin_id and move onto next step
    #if goal is None:
      #goal = parceldetector.readimage(detector=detector, camera=camera) # this function has its own while loop that will run infinitely until qr is detected

    # print("test")
    # break

    # 2. Begin moving to bin
      # PROCESS 1
      # Drive for x amount of seconds at full frequency (distance = vel * x)
        # Continuously update robot position using dt (can be obtained using time.sleep) and velocity
          # Note that we will need:
            # speed of wheel_x and wheel_y independently 
            # dt
          # This will allow us to calculate wheel_x and wheel_y linear displacement
          # Estimate linear position based on this
        
        # PROCESS 2
        # Continuously poll ultrasonics
          # For obstacle detection
            # If obstacle detected (so distance < threshold) go into obstacle detection code
              # This requires us to stop drive, record time moved and estimated x,y (do we need time moved if we have x,y?)
          # For checking if robot is at reasonable distance from the wall
    goal = (0,1)
    customRobot.reset_x_y() # for now we'll reset robot's estimated x,y to 0,0 everytime we load a package, but could use ulltrasonic and previous history to reset it properly
    trans_duration_x, trans_duration_y = customRobot.drive_to_goal(goal=goal)

    customRobot.deploy_packages() # could also only trigger this once limit switch

    # 3. Drive back to start
    customRobot.drive_to_origin(trans_duration_x, trans_duration_y)

if __name__ == "__main__":
  main()
  # main_process = multiprocessing.Process(target=main) # so continuously drive out bot
  # #sensor_process = multiprocessing.Process(target=customRobot.ultrasonic_sensor) # continuously check sensors

  # main_process.start()
  # #sensor_process.start()

  # main_process.join()
  # #sensor_process.join()

