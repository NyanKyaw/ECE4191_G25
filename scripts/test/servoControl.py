from gpiozero import Servo
from time import sleep
import config
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
LEVER = 22
GPIO.setup(LEVER, GPIO.OUT)
servo = Servo(LEVER)
servo.min()

def toggleServo(servo):
	servo.value = -1*servo.value #inverts the position of the servo
		

def deployParcel(servo):
    toggleServo(servo)
    print(servo.value)
    sleep(2) #wait for two seconds
    toggleServo(servo)
    print(servo.value)
    
if __name__ == "__main__":
    deployParcel(servo)