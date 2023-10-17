# Script for testing basic navigation
# Integrated with limit switch
import RPi.GPIO as GPIO
from servoTest import test_servo
from testingcam import run_camera
import serial
import time

LIMIT_SWITCH = 23
LEVER = 22
alpha = 1.3



def pinsetup():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LIMIT_SWITCH, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
	GPIO.setup(LEVER, GPIO.OUT)

def test_limit_switch():
	ls_state = GPIO.input(LIMIT_SWITCH)
	if (ls_state == GPIO.HIGH):
		print("LS High!")
	else:
		print("LS Low")


def translate(distance):
	"""
	This function is used for all translational movements except when reversing to bin.
	
	Args:
		distance in cm - positive for forwards, negative for reverse
	
	Returns:
		None
	"""
	
	#distance to time conversion
	set_time = alpha*(5/53.014)*distance
	print(set_time)
	
	ser = serial.Serial('/dev/ttyUSB0', 9600)
	time.sleep(2)
	
	if (distance>0):
		val_str = "-0.5+0.5\n"
	elif (distance<0):
		val_str = "+0.5-0.5\n"
	ser.write(val_str.encode())
	time.sleep(set_time) # For Forward/Backward
	
	stop = f"{0}\n"
	ser.write(stop.encode())
	ser.close()
	
def rotate(angle):
	"""
	This function is used for all rotational movements.
	
	Args:
		angle in deg - positive for clkwise, negative for anti-clkwise
	
	Returns:
		None
	"""
	ser = serial.Serial('/dev/ttyUSB0', 9600)
	time.sleep(2)
	set_time = (5.56 /360)*abs(angle)
	
	if (angle>0):
		val_str = "+0.5+0.5\n" #Rotate Right
	elif (angle<0):
		val_str = "-0.5-0.5\n" #Rotate Left
		
	ser.write(val_str.encode())
	time.sleep(set_time) # For Forward/Backward
	
	stop = f"{0}\n"
	ser.write(stop.encode())
	ser.close()
	

def reverse_to_bin():
	"""
	This function is used only used when reversing to bin as it uses a limit switch stop.
	
	Args:
		None
	
	Returns:
		None
	"""
	ls_state = GPIO.input(LIMIT_SWITCH)
	ser = serial.Serial('/dev/ttyUSB0', 9600)
	time.sleep(2)
	while (ls_state == GPIO.LOW):
		val_str = "+0.5-0.5\n"
		ser.write(val_str.encode())	
		ls_state = GPIO.input(LIMIT_SWITCH)
	
	#time.sleep(0)
	stop = f"{0}\n"
	ser.write(stop.encode())
	ser.close()

	
def bin_A():
	"""
	Directions to Bin A
	"""
	# Waypoint 1
	forward(81.98)
	rotate(131.186)
	
	#Waypoint 2
	translate(-9.37)
	rotate(48.814)
	
	#To Bin A
	reverse_to_bin()
	
def bin_B():
	"""
	Directions to Bin B
	"""
	# Waypoint 1
	forward(81.98)
	rotate(90)
	
	#Waypoint 2
	translate(36.8)
	rotate(90)
	
	#To Bin A
	reverse_to_bin()
	
def bin_C():
	"""
	Directions to Bin C
	"""
	# Waypoint 1
	forward(81.98)
	rotate(90)
	
	#Waypoint 2
	translate(69.8)
	rotate(117.35)
	
	#Waypoint 3
	translate(-12.53)
	rotate(-27.35)
	
	#To Bin A
	reverse_to_bin()

def main():
	pinsetup()
	try:
		# Pathway to Bin B
		#forward(83)
		#rotate(90)
		# forward(36.8)
		# rotate(-90)

		# Testing Lever Integration
		#backwards()
		test_servo()
		
		# Testing Camera
		#run_camera()
		
	except Exception as e:
		print("Error")
		
	GPIO.cleanup()

if __name__ == "__main__":
	main()
	


