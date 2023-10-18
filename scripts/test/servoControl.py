from gpiozero import Servo
from time import sleep
import config


def toggleServo(servo):
	servo.value = -1*servo.value #inverts the position of the servo
		

def deployParcel(servo):
    toggleServo(servo)
    time.sleep(2) #wait for two seconds
    toggleServo(servo)
    
