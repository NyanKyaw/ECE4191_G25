import RPi.GPIO as GPIO
import utility.config as config
import utility.utility as utils
import utility.parceldetector as parceldetector
from scripts.main.robot import CustomRobot
import math
import numpy as np
import multiprocessing

def main():
  left_motor_pins, right_motor_pins, left_encoder_pins, right_encoder_pins = config.pinsetup()

  detector, camera = config.camerasetup()

  customRobot = CustomRobot(left_motor_pins, right_motor_pins, left_encoder_pins, right_encoder_pins)

  goal = None
  # PROCESS 1
  while True: # infinite loop
    if goal is None:
      goal = parceldetector.readimage(detector=detector, camera=camera) # this function has its own while loop that will run infinitely until qr is detected

    print("test")
    break
    # 1. check camera for QR code detection
      # if not detected detected, continue
      # if detected, return bin_id and move onto next step

    # 2. Begin moving to bin
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

if __name__ == "__main__":
  main_process = multiprocessing.Process(target=main) # so continuously drive out bot
  #sensor_process = multiprocessing.Process(target=customRobot.ultrasonic_sensor) # continuously check sensors

  main_process.start()
  #sensor_process.start()

  main_process.join()
  #sensor_process.join()

