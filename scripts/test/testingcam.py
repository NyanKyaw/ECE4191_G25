import RPi.GPIO as GPIO
import config as config
import utility as utils
import parceldetector as parceldetector

detector, camera = config.camerasetup()

goal = None

while True: # infinite loop
	if goal is None:
		goal = parceldetector.readimage(detector=detector, camera=camera) # this function has its own while loop that will run infinitely until qr is detected

