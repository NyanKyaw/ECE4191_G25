import RPi.GPIO as GPIO
import config as config
import utility as utils
import parceldetector as parceldetector

def run_camera():
	detector, camera = config.camerasetup()

	goal = None

	while goal == None: # infinite loop
		goal = parceldetector.readimage(detector=detector, camera=camera) # this function has its own while loop that will run infinitely until qr is detected
			
	print(goal)
			
	return goal

if __name__ == "__main__":
	run_camera()
