# Script for testing basic navigation
# Integrated with limit switch
import RPi.GPIO as GPIO
from servoTest import run_servo
from parceldetector import run_camera
import serial
import time

LIMIT_SWITCH = 23
LEVER = 22
tuning_param_trans = 1.3
tuning_param_rotate = 1.1

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
	set_time = abs(tuning_param_trans*(5/53.014)*distance)
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
	set_time = abs(tuning_param_rotate*(5.56 /360)*abs(angle))
	
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
	This function is used only used when reversing to bin as it uses a limit switch stop
	
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

def reverse_to_loading_zone():
	""" 
	Robot reverses to the loading zone, uses P wall-distance controller
	"""
	ls_state = GPIO.input(LIMIT_SWITCH)
	ser = serial.Serial('/dev/ttyUSB0', 9600)
	time.sleep(2)
	while (ls_state == GPIO.LOW):
		val_str = "+0.5-0.5\n" #add P Wall-distance controller
		ser.write(val_str.encode())	
		ls_state = GPIO.input(LIMIT_SWITCH)
	
	#time.sleep(0) - check on wed to add
	stop = f"{0}\n"
	ser.write(stop.encode())
	ser.close()
	
def bin_A():
	"""
	Directions to Bin A
	"""
	# Waypoint 1
	print("1")
	rotate(131.186)
	print(2)
	
	#Waypoint 2
	translate(-9.37)
	print(3)
	rotate(48.814)
	print(4)
	
	#To Bin A
	reverse_to_bin()
	print(5)
	
def back_from_bin_A():
	"""
	Directions from Bin A to Loading Zone
	"""
	# Waypoint 2
	translate(7.9825)
	rotate(41.186)
	
	#Waypoint 1
	translate(9.37)
	rotate(-131.86)
	
	#Loading Zone
	reverse_to_loading_zone()
	
	
	
	
def bin_B():
	"""
	Directions to Bin B
	"""
	# Waypoint 1
	translate(81.98)
	rotate(90)
	print(1)
	
	#Waypoint 2
	translate(36.8)
	rotate(90)
	print(2)
	
	#To Bin A
	reverse_to_bin()
	print(3)
	
def back_from_bin_B():
	"""
	Directions from Bin A to Loading Zone
	"""
	# Waypoint 2
	translate(13.98)
	rotate(90)
	
	# Wapoint 1
	translate(36.8)
	rotate(90)
	
	# Loading Zone
	reverse_to_loading_zone()
	
	
def bin_C():
	"""
	Directions to Bin C
	"""
	# Waypoint 1
	translate(81.98)
	rotate(90)
	print(1)
	
	#Waypoint 2
	translate(69.8)
	rotate(117.35)
	print(2)
	
	#Waypoint 3
	translate(-12.53)
	rotate(-27.35)
	print(3)
	
	#To Bin A
	reverse_to_bin()
	print(4)

def main():
	pinsetup()
	while True:
		goal_bin = None
		print("Waiting for QR Code")
		while goal_bin == None:
			goal_bin = run_camera()
			
		if (goal_bin == "A"):
			bin_A()
		elif (goal_bin == "B"):
			bin_B()
		elif (goal_bin == "C"):
			bin_C()
		else:
			print("Unknown Bin")
			
		print(6)
		run_servo()
		print(7)
		
	GPIO.cleanup()

if __name__ == "__main__":
	main()
	


