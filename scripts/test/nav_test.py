# Script for testing basic navigation
# Integrated with limit switch
import RPi.GPIO as GPIO
from servoTest import run_servo
from parceldetector import run_camera
import serial
import time
import multiprocessing
import config

LIMIT_SWITCH_1 = 23
LIMIT_SWITCH_2 = 25
LEVER = 22
tuning_param_trans = 1
tuning_param_rotate = 0.95

def pinsetup():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LIMIT_SWITCH_1, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
	GPIO.setup(LIMIT_SWITCH_2, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
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

	update_velocities(set_time=set_time)
	
	stop = f"{0}\n"
	ser.write(stop.encode())
	ser.close()

def read_ultrasonics(pin0, pin1, pin2, pin3, pin4):
	TRIG_1, ECHO_1, ECHO_2, ECHO_3, ECHO_4 = pin0, pin1, pin2, pin3, pin4
	while True:
		# Send trigger
		GPIO.output(TRIG_1, False)
		time.sleep(0.1) #0.2
		GPIO.output(TRIG_1, True)
		time.sleep(0.2) #1
		GPIO.output(TRIG_1, False)

		while GPIO.input(ECHO_1)==0:
			pulse_start_1=time.time()
		while GPIO.input(ECHO_1)==1:
			pulse_end_1 = time.time()
		pulse_duration_1=pulse_end_1-pulse_start_1
		distance_1=pulse_duration_1*17150
		print("US 1 Distance:", distance_1)
			
		#while GPIO.input(ECHO_2)==0:
		#	pulse_start_2=time.time()
		#while GPIO.input(ECHO_2)==1:
		#	pulse_end_2=time.time()
		#pulse_duration_2=pulse_end_2-pulse_start_2
		#distance_2=pulse_duration_2*17150
		#print("US 2 Distance:", distance_2)

		#while GPIO.input(ECHO_3)==0:
		#	print("stuck")
		#	pulse_start_3=time.time()
		#while GPIO.input(ECHO_3)==1:
		#	print("high")
		#	pulse_end_3=time.time()
		#pulse_duration_3=pulse_end_3-pulse_start_3
		#distance_3=pulse_duration_3*17150
		#print("US 3 Distance:", distance_3)

		#while GPIO.input(ECHO_4)==0:
		#	pulse_start=time.time()
		#while GPIO.input(ECHO_4)==1:
		#	pulse_end=time.time()
		#pulse_duration=pulse_end-pulse_start
		#distance_4=pulse_duration*17150
		#print("US 4 Distance:", distance_4)

def update_velocities(set_time):
	start = time.time()
	p_val = 0.02
	while time.time() - start < set_time:
		continue
	
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
	ls_state_1 = GPIO.input(LIMIT_SWITCH_1)
	ls_state_2 = GPIO.input(LIMIT_SWITCH_2)
	ser = serial.Serial('/dev/ttyUSB0', 9600)
	time.sleep(2)
	val_str = "+0.5-0.5\n"
	ser.write(val_str.encode())	
	while (ls_state_1 == GPIO.LOW or ls_state_2 == GPIO.LOW):
		ls_state_1 = GPIO.input(LIMIT_SWITCH_1)
		ls_state_2 = GPIO.input(LIMIT_SWITCH_2)
	
	#time.sleep(0)
	stop = f"{0}\n"
	ser.write(stop.encode())
	ser.close()



# def reverse_to_loading_zone():
# 	""" 
# 	Robot reverses to the loading zone, uses P wall-distance controller
# 	"""
# 	ls_state = GPIO.input(LIMIT_SWITCH)
# 	ser = serial.Serial('/dev/ttyUSB0', 9600)
# 	time.sleep(2)
# 	val_str = "+0.5-0.5\n" #add P Wall-distance controller
# 	ser.write(val_str.encode())	
# 	while (ls_state == GPIO.LOW):	
# 		ls_state = GPIO.input(LIMIT_SWITCH)
# 		print(ls_state)
	
# 	#time.sleep(0) - check on wed to add
# 	stop = f"{0}\n"
# 	ser.write(stop.encode())
# 	ser.close()
	
def bin_A():
	"""
	Directions to Bin A
	"""
	# Waypoint 1
	translate(81.98)
	rotate(131.186)
	print(2)
	
	#Waypoint 2
	translate(-9.37)
	print(3)
	rotate(48.814)
	print(4)
	
	#To Bin A
	reverse_to_bin()
	run_servo()
	print(5)
	
def back_from_bin_A():
	"""
	Directions from Bin A to Loading Zone
	"""
	# Waypoint 2
	translate(7.9825)
	rotate(-41.186)
	
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
	run_servo()
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
	ser = serial.Serial('/dev/ttyUSB0', 9600)
	time.sleep(2)

	stop = f"{0}\n"
	ser.write(stop.encode())
	ser.close()
	#Waypoint 1
	# Waypoint 1
	# translate(81.98)
	# rotate(90)
	# print(1)
	
	# #Waypoint 2
	# translate(73.8)
	# rotate(117.35) # removed 1 cm
	# print(2)
	
	# #Waypoint 3
	# translate(-13.53)
	# rotate(-27.35)
	# # print(3)
	
	# #To Bin A
	# reverse_to_bin()
	# run_servo()


		
	GPIO.cleanup()

if __name__ == "__main__":
	left_motor_pins, right_motor_pins, left_encoder_pins, right_encoder_pins, pwm_left, pwm_right, ultrasonic_pins = config.pinsetup()
	print(ultrasonic_pins)


	main_process = multiprocessing.Process(target=main) # so continuously drive out bot
	sensor_process = multiprocessing.Process(target=read_ultrasonics, args=(ultrasonic_pins)) # continuously check sensors

	main_process.start()
	sensor_process.start()

	main_process.join()
	sensor_process.join()
		


