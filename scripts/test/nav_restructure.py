#Attempt to restructure the navigation
#Written: Grant Bradbeer 18/10/23
#Last Updated: Grant Bradbeer 18/10/23
import RPi.GPIO as GPIO
from gpiozero import Servo
from servoTest import run_servo
from parceldetector_restructure import run_camera
import serial
import time
import multiprocessing
import config

import numpy as np



GPIO.setmode(GPIO.BCM)
LIMIT_SWITCH_1 = 23
LIMIT_SWITCH_2 = 25
LEVER = 22

GPIO.setup(LIMIT_SWITCH_1, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(LIMIT_SWITCH_2, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(LEVER, GPIO.OUT)
#Initialising limit switches
ls_state_1 = GPIO.input(LIMIT_SWITCH_1)
ls_state_2 = GPIO.input(LIMIT_SWITCH_2)

# Ultrasonic set up
GPIO.setwarnings(False)

#Ultrasonic Sensor No. 1 - Side left
TRIG_1 = 27
ECHO_1 = 2
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_1, GPIO.OUT)
GPIO.setup(ECHO_1, GPIO.IN)

#Ultrasonic Sensor No. 2 - Front left
ECHO_2 = 3

GPIO.setmode(GPIO.BCM)
GPIO.setup(ECHO_2, GPIO.IN)

#Ultrasonic Sensor No. 3 - Front centre
ECHO_3 = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(ECHO_3, GPIO.IN)

#Ultrasonic Sensor No. 4 - Front right
ECHO_4 = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(ECHO_4, GPIO.IN)

#Ultrasonic Sensor No. 5- Side right
ECHO_5 = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(ECHO_5, GPIO.IN)

ser = serial.Serial('/dev/ttyUSB0', 9600)
time.sleep(2)

obstacle_threshold = 10

	#return TRIG_1, ECHO_1, ECHO_2, ECHO_3, ECHO_4, ECHO_5, ls_state_1, ls_state_2

def computeTrajectory(location,orientation,waypoint):
	distanceTranslation = np.sqrt((waypoint[0]-location[0])**2 + (waypoint[1]-location[1])**2)*np.power(-1,waypoint[6])			
	#Euclidian distance to the waypoint.
	#Could probably do this ^ with np.linalg.norm if we wanted
	print("In Compute Trajectory")
	print(f"Location {location}")
	print(f"Orientation {location}")
	print(f"Next Waypoint: {waypoint[0]}, {waypoint[1]}")
	angleRotation = np.degrees(np.arctan2((waypoint[0]-location[0]),(waypoint[1]-location[1]))) +180*waypoint[6]
	
	#NOTE: We may need some sort of angle constant eg +90 to convert this to our coordinate system. Not sure
	angleRotation =  angleRotation - orientation #This gives us the angle we need to rotate
	if abs(angleRotation)>180:
		angleRotation = angleRotation-360*(abs(angleRotation)/angleRotation)
	print(f"Angle: {angleRotation}")

	return distanceTranslation, angleRotation

def readUltrasonic(ECHO_NUM):
	"""
		INPUT THE PINS FOR THE CORRESPONDING ULTRASONIC SENSOR AND THIS FUNCTION WILL RETURN THE DISTANCE READING
		"""
	smoothing_factor = 10
	dist_sum = 0
	count = 0
	for x in range(smoothing_factor):
		GPIO.output(TRIG_1, False)
		time.sleep(0.1)
		GPIO.output(TRIG_1, True)
		time.sleep(0.2)
		GPIO.output(TRIG_1, False)
		while GPIO.input(ECHO_NUM)==0:
			pulse_start=time.time()
			#print(4.1)
		while GPIO.input(ECHO_NUM)==1:
			pulse_end=time.time()
			#print(4.2)
		pulse_duration=pulse_end-pulse_start
		distance=pulse_duration*17150

		count+=1
		dist_sum += distance

		if count == smoothing_factor:
			dist_average = dist_sum/smoothing_factor
			# Reset values
			count = 0
			return dist_average
		
def readUltrasonic3():
	"""
		INPUT THE PINS FOR THE CORRESPONDING ULTRASONIC SENSOR AND THIS FUNCTION WILL RETURN THE DISTANCE READING
		"""
	smoothing_factor = 1
	dist_sum = 0
	count = 0
	for x in range(smoothing_factor):
		GPIO.output(TRIG_1, False)
		time.sleep(0.1)
		GPIO.output(TRIG_1, True)
		time.sleep(0.2)
		GPIO.output(TRIG_1, False)
		while GPIO.input(ECHO_3)==0:
			pulse_start=time.time()
			#print(4.1)
		while GPIO.input(ECHO_3)==1:
			pulse_end=time.time()
			#print(4.2)
		pulse_duration=pulse_end-pulse_start
		distance=pulse_duration*17150

		count+=1
		dist_sum += distance

		if count == smoothing_factor:
			dist_average = dist_sum/smoothing_factor
			# Reset values
			count = 0
			return dist_average

def reverseToLimitSwitch():
	"""
	This function is used only used when reversing to bin as it uses a limit switch stop
	
	Args:
		None
	
	Returns:
		None
	"""
	print("Running until limit switch is triggered")
	LIMIT_SWITCH_1 = 23
	LIMIT_SWITCH_2 = 25

	
	GPIO.setup(LIMIT_SWITCH_1, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
	GPIO.setup(LIMIT_SWITCH_2, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
	GPIO.setup(LEVER, GPIO.OUT)
	#Initialising limit switches
	# ls_state_1 = GPIO.input(LIMIT_SWITCH_1)
	# ls_state_2 = GPIO.input(LIMIT_SWITCH_2)
	
	# ser = serial.Serial('/dev/ttyUSB0', 9600)
	# time.sleep(2)
	val_str = "+0.5-0.5\n"
	ser.write(val_str.encode())	
	ls_state_1 = GPIO.input(LIMIT_SWITCH_1)
	ls_state_2 = GPIO.input(LIMIT_SWITCH_2)
	while (ls_state_1 == GPIO.LOW or ls_state_2 == GPIO.LOW):
		ls_state_1 = GPIO.input(LIMIT_SWITCH_1)
		ls_state_2 = GPIO.input(LIMIT_SWITCH_2)
	
	#time.sleep(0)
	stop = f"{0}\n"
	ser.write(stop.encode())
	# ser.close()
	
def stop():
	
	stop = f"{0}\n"
	ser.write(stop.encode())
	

def checkWallDistance(sideToCheck):
	#0 - check the left ultrasonics
	start_time = time.time()
	while True:
		if(sideToCheck==0):
			dist1 = readUltrasonic(ECHO_1)
			print(f"Dist1: {dist1}")
			dist2 = readUltrasonic(ECHO_2)
			print(f"Dist2: {dist2}")
		#1 - check the right ultrasonics
			current_time = time.time()
			avgDistance = (dist1 + dist2)/2
		else:
			dist1 = readUltrasonic(ECHO_3)
			print(f"Dist1: {dist1}")
			dist2 = readUltrasonic(ECHO_4)
			print(f"Dist2: {dist2}")

			current_time = time.time()
			avgDistance = (dist1 + dist2)/2
			
		
		if (abs(dist1-dist2)<0.5):

			print(f" Average wall dist: {avgDistance}")
			return avgDistance
		
		if(current_time-start_time>10):
			print("10 Second Limit Met")
			print(f" Average wall dist: {avgDistance}")
			return avgDistance # maybe hard code to 1.5
def rotate(angle):
	"""
	This function is used for all rotational movements.
	
	Args:
		angle in deg - positive for clkwise, negative for anti-clkwise
	
	Returns:
		None
	"""
	if angle == 0:
		return

	# ser = serial.Serial('/dev/ttyUSB0', 9600)
	# time.sleep(2)
	set_time = abs(tuning_param_rotate*(5.14/360)*abs(angle))
	
	if (angle>0):
		val_str = "+0.5+0.5\n" #Rotate Right
		print(f"Rotate: {val_str}")
	elif (angle<0):
		val_str = "-0.5-0.5\n" #Rotate Left
		print(f"Rotate: {val_str}")
		
	ser.write(val_str.encode())
	time.sleep(set_time) # For Forward/Backward

	stop = f"{0}\n"
	ser.write(stop.encode())
	# ser.close()
	

def translate(distance):
	"""
	This function is used for all translational movements except when reversing to bin.
	
	Args:
		distance in cm - positive for forwards, negative for reverse
	
	Returns:
		None
	"""
	if distance == 0:
		return
	#distance to time conversion
	set_time = abs(tuning_param_trans*(5/53.014)*distance)
	print(f"Set Time in Translate_func: {set_time}")
	
	# ser = serial.Serial('/dev/ttyUSB0', 9600)
	# time.sleep(2)
	
	if (distance>0):
		val_str = "-0.5+0.5\n"
	elif (distance<0):
		val_str = "+0.5-0.5\n"
	ser.write(val_str.encode())
	start_time = time.time()
	current_time = time.time()
	#obstacle_detection_process = multiprocessing.Process(target=detect_obstacle())
	#obstacle_detection_process.start()
	#obstacle_detection_process.join()
	delay_time = 0
	while current_time-start_time < set_time + delay_time: 
		current_time = time.time()
		
		stop_time = detect_obstacle()
		if stop_time is not None:
			delay_time = time.time() - stop_time
			print(f"Delay Time {delay_time}")
			ser.write(val_str.encode())
		#if stop_drive_event.is_set():
			# stop_time = time.time()
			# stop = f"{0}\n"
			# ser.write(stop.encode())
			# while stop_drive_event.is_set():
			# 	pass
			# delay_time = time.time() - stop_time
		#print(current_time-start_time)
		#detect_obstacle()
	
	#obstacle_detection_process.terminate()

	stop = f"{0}\n"
	ser.write(stop.encode())
	# ser.close()

def detect_obstacle():
	distance = readUltrasonic3()
	if distance <= obstacle_threshold:
		print("OBSTACLE DETECTED")
		#stop_drive_event.set()
		stop_time = time.time()
		stop = f"{0}\n"
		ser.write(stop.encode())

		while distance <= 10:
			print(f"US 3 distance: {distance}")
			distance = readUltrasonic3()
		return stop_time
		#stop_drive_event.clear()	
	return None

def main():
	dist_from_wall_B = 16

	bin_A = [[23.2,94,0,0,0,0,0],[dist_from_wall,100,0,0,0,0, 1], [dist_from_wall,120,0,0,1, 1, 1], [dist_from_wall,100,0,0,0, 0, 0], [23.2,94,0,0,0, 0, 0], [23.2, 12.02, 1, 0, 1, 0, 1]] 
	bin_B = [[23.2,94,0,0,0,0,0],[60,94,0,0,0,0,0],[60,120,0,0,1,1,1], [60,94,0,0,0,0,0], [23.2,94,0,0,0,0,0], [23.2, 12.02, 1, 0, 1, 0, 1]]
	bin_C = [[33.2,94,0,0,0,0,0],[93,94,0,0,0,0,0],[120-dist_from_wall_B,100,0,0,0,0,1], [120-dist_from_wall_B,120,0,0,1,1,1], [120-dist_from_wall_B,100,0,0,0,0,0], [93,94,0,0,0,0,0], [33.2,94,0,0,0,0,0], [33.2, 12.02, 1, 0, 1, 0,1]]
	waypoints = []
    #waypoints = bin_B
	#TRIG_1, ECHO_1, ECHO_2, ECHO_3, ECHO_4, ECHO_5, ls_state_1, ls_state_2 = pinsetup()
	location = [33.2,12.0175] #x = 23.2cm, y = 12.0175cm
	orientation = 0 #degrees clockwise from north, we might need to use this in radians for numpy
	

	while True:
		if len(waypoints) == 0:
			waypoints = run_camera()
		else:
			#Defining a temporary location variable - this can be located somewhere else or in another class just putting it here so it's easy to read
			
			#Defining an empty waypoints vector
			
			
			#while no waypoints, check the camera	

			# For now ommitted until QT depencies on mine
			# while(len(waypoints) == 0):
			# 	goal = run_camera()
			

			# #Read first waypoint
			nextWaypoint = waypoints[0] #should be a row vector of length 5
			print(f"Next Waypoint: {nextWaypoint}")


			#Check for the reverse and deploy flag
			#IF REVERSE FLAG IS HIGH, ALL OTHER PROPERTIES OF THE WAYPOINTS ARE IGNORED, SO DONT PUT USEFUL COORDINATE INFO, JUST DUPLICATE THE PREVIOUS ONE
			distance, angle = computeTrajectory(location,orientation,nextWaypoint)
			print(f"Distance: {distance}")
			print(f"Angle: {angle}")
			#rotate heading
			rotate(angle)
			orientation = (orientation + angle)%360 #updating orientation. Please double check my sign here
			print(f"Orientation: {orientation}")
			time.sleep(1) #might not need

			if(nextWaypoint[4]==1):
				reverseToLimitSwitch()
				#Depending on if we have reversed into the south or north wall, we may deploy, and how we update position changes
				if(nextWaypoint[5]==1):
					#check wall distance and update the robots X coordinate
					# STOP CALIBRATION
					# print("Wall distance to the right")
					# print(checkWallDistance(1))
					# print("Wall distance to the left")
					# print(checkWallDistance(0))

					
					# xr = 10.375 + checkWallDistance(nextWaypoint[3]) #the distance from the center of the robot to the wall		
					# location[0] =  (nextWaypoint[3])*(xr) + (1-nextWaypoint[3])*(120-xr) #computes differently depending on the side that the wall is on
					# #y coordinate will always be the same when in contact with the North wall
					# location[1] = 120-11.265
					location[0] = nextWaypoint[0]
					location[1] = nextWaypoint[1]
					#updating orientation
					orientation = 180 #since we will now always be facing south
					run_servo(servo)#deploy parcel function from servoControl.py
					#This will be the case when deploying in bins A and C but not B
					print(f"x: {location[0]} y: {location[1]} orientation: {orientation}")

				else:
					#At loading zone, update location
					xr = 10.375 + checkWallDistance(nextWaypoint[3]) #the distance from the center of the robot to the wall
					location[0] =  (1-nextWaypoint[3])*(xr) + (nextWaypoint[3])*(120-xr) #computes differently depending on the side that the wall is on
					#y coordinate will always be the same when in contact with the North wall
					location[1] = 11.265
					orientation = 0
					print(f"x: {location[0]} y: {location[1]} orientation: {orientation}")
					translate(10)
					location[1] += 10
					print(f"x: {location[0]} y: {location[1]} orientation: {orientation}")
					

			

			else:
				#Compute trajectory of the waypoint
				translate(distance)
				
				#drive to waypoint
				

				#conditional location update
				#if the calibrate flag is high, meaning we can measure using ultrasonics
				#note that since we handle the reverse and deploy seperately, if we can measure here, it will always be on the south
				#side of the arena
				if(nextWaypoint[2]==1):
					#side dependant on an element of the vector
					#use the checkDistance function
					xr = 10.375 + checkWallDistance(nextWaypoint[3]) #the distance from the center of the robot to the wall
					location[0] =  (nextWaypoint[3])*(120-xr) + (1-nextWaypoint[3])*(xr) #computes differently depending on the side that the wall is on
					#y coordinate will always be the same when in contact with the South wall
					location[1] = 11.265
					#updating orientation
					orientation = 0 #since we will now always be facing north
				
				#if not update the current location to be the waypoint
				else:
					location = [nextWaypoint[0],nextWaypoint[1]]

			waypoints.pop(0)
			print(f"Waypoints Remaining: {waypoints}")

	ser.close()

if __name__ == "__main__":
		
	#Defining pins and constants
	tuning_param_trans = 1
	#tuning_param_rotate = 1.01
	tuning_param_rotate = 0.96
	dist_from_wall = 18 # Can delete when camera function is brought back

	#Setting up servo - this can go elsewhere if necessary
	servo = Servo(LEVER)
	servo.min() #sets the servo to min on startup. can go elsewhere

	#Defining ultrasonic pins
	ULTRASONIC_PINS = [12,13,14,15,16] #CHANGE THESE THEY ARE RANDOM NUMBERS RIGHT NOW
	TRIGGER_PIN = 11 #ALSO A RANDOM PIN NUMBER
	#WHEN REFERRING TO ULTRASONICS IN THE REMAINDER OF THE CODE, ASSUMING THAT BACK LEFT IS ULTRASONIC 0 AND IT PROCEEDS CLOCKWISE

	#Defining a temporary location variable - this can be located somewhere else or in another class just putting it here so it's easy to read
	location = [23.2,11.265] #x = 23.2cm, y = 12.0175cm
	orientation = 0 #degrees clockwise from north, we might need to use this in radians for numpy

	#Defining an empty waypoints vector
	waypoints = []
	stop_drive_event = multiprocessing.Event()
	stop_drive_event.clear()

	#stop()
	#reverseToLimitSwitch()
	
	#checkWallDistance(0)

	main()
	# obstacle_detection_process = multiprocessing.Process(target=detect_obstacle())
	# mains_prsocess = multiprocessing.Process(target=main())

	# obstacle_detection_process.start()
	# main_process.start()

	# obstacle_detection_process.join()
	# main_process.join()



"""
waypoints = [[23,80,0,0,0,0,0],[50,30,0,0,0,0, 0], [23.2,20,0,0,0, 0]]
	while len(waypoints)>0:
		distance, angle = computeTrajectory(location,orientation,waypoints[0])
		print(distance)
		print(angle)
		#rotate heading
		rotate(angle)
		orientation = (orientation + angle)%360 #updating orientation. Please double check my sign here
		time.sleep(1) #might not need
		translate(distance)
		location = [waypoints[0][0],waypoints[0][1]]
		waypoints.pop(0)
		"""