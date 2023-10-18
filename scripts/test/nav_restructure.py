#Attempt to restructure the navigation
#Written: Grant Bradbeer 18/10/23
#Last Updated: Grant Bradbeer 18/10/23
import RPi.GPIO as GPIO
from gpiozero import Servo
from servoTest import run_servo
from parceldetector import run_camera
import serial
import time
import multiprocessing
import config

import numpy as np

#Defining pins and constants
GPIO.setmode(GPIO.BCM)
LIMIT_SWITCH_1 = 23
LIMIT_SWITCH_2 = 25
LEVER = 22
tuning_param_trans = 1
tuning_param_rotate = 0.95

#Setting up servo - this can go elsewhere if necessary
servo = Servo(22)
servo.min() #sets the servo to min on startup. can go elsewhere

#Defining ultrasonic pins
ULTRASONIC_PINS = [12,13,14,15,16] #CHANGE THESE THEY ARE RANDOM NUMBERS RIGHT NOW
TRIGGER_PIN = 11 #ALSO A RANDOM PIN NUMBER
#WHEN REFERRING TO ULTRASONICS IN THE REMAINDER OF THE CODE, ASSUMING THAT BACK LEFT IS ULTRASONIC 0 AND IT PROCEEDS CLOCKWISE

#Defining a temporary location variable - this can be located somewhere else or in another class just putting it here so it's easy to read
location = [23.2,12.0175] #x = 23.2cm, y = 12.0175cm
orientation = 0 #degrees clockwise from north, we might need to use this in radians for numpy

#Defining an empty waypoints vector
waypoints = []

def pinsetup():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LIMIT_SWITCH_1, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
	GPIO.setup(LIMIT_SWITCH_2, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
	GPIO.setup(LEVER, GPIO.OUT)
	#Initialising limit switches
	ls_state_1 = GPIO.input(LIMIT_SWITCH_1)
	ls_state_2 = GPIO.input(LIMIT_SWITCH_2)

def computeTrajectory(location,orientation,waypoint):
	distanceTranslation = np.sqrt((waypoint[0]-location[0])**2 + (waypoint[1]-location[1])**2) 					
	#Euclidian distance to the waypoint.
	#Could probably do this ^ with np.linalg.norm if we wanted
	angleRotation = np.degrees(np.arctan2((waypoint[0]-location[0])/(waypoint[1]-location[1])))
	#NOTE: We may need some sort of angle constant eg +90 to convert this to our coordinate system. Not sure
	angleRotation = orientation - angleRotation #This gives us the angle we need to rotate

	return distanceTranslation, angleRotation

def readUltrasonic():
	"""
		INPUT THE PINS FOR THE CORRESPONDING ULTRASONIC SENSOR AND THIS FUNCTION WILL RETURN THE DISTANCE READING
		"""
	dist_sum = 0
	count = 0
	while True:	
		GPIO.output(TRIG_1, False)
		time.sleep(0.1)
		GPIO.output(TRIG_1, True)
		time.sleep(0.2)
		GPIO.output(TRIG_1, False)
		while GPIO.input(ECHO_5)==0:
			pulse_start=time.time()
			print(5.1)
		while GPIO.input(ECHO_5)==1:
			pulse_end=time.time()
			print(5.2)
		pulse_duration=pulse_end-pulse_start
		distance=pulse_duration*17150

		for x in range(32000):
			count+=1
			dist_sum += distance

		if count == 32000:
			dist_average = dist_sum/32000
			
			# Reset values
			count = 0
			dist_sum=0
			return dist_average

def reverseToLimitSwitch():
	"""
	This function is used only used when reversing to bin as it uses a limit switch stop
	
	Args:
		None
	
	Returns:
		None
	"""
	
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
	
def stop():
	ser = serial.Serial('/dev/ttyUSB0', 9600)
	time.sleep(2)
	stop = f"{0}\n"
	ser.write(stop.encode())
	ser.close()

def checkWallDistance(sideToCheck):
    	#0 - check the left ultrasonics
    	if(sideToCheck==0):
        	dist1 = readUltrasonic(ultrasonic_pins[0])
        	dist2 = readUltrasonic(ultrasonic_pins[1])
    	#1 - check the right ultrasonics
    	else:
        	dist1 = readUltrasonic(ultrasonic_pins[3])
        	dist2 = readUltrasonic(ultrasonic_pins[4])

    	avgDistance = np.mean(dist1,dist2)

    	return avgDistance

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
	set_time = abs(tuning_param_rotate*(5.14/360)*abs(angle))
	
	if (angle>0):
		val_str = "+0.5+0.5\n" #Rotate Right
	elif (angle<0):
		val_str = "-0.5-0.5\n" #Rotate Left
		
	ser.write(val_str.encode())
	time.sleep(set_time) # For Forward/Backward

	stop = f"{0}\n"
	ser.write(stop.encode())
	ser.close()
	

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

def main():
	pinsetup()
	#while no waypoints, check the camera	

	while(len(waypoints) == 0):
		run_camera()
		
	#Read first waypoint
	nextWaypoint = waypoints[0] #should be a row vector of length 5
	print(nextWaypoint)
"""
	    #Check for the reverse and deploy flag
	    #IF REVERSE FLAG IS HIGH, ALL OTHER PROPERTIES OF THE WAYPOINTS ARE IGNORED, SO DONT PUT USEFUL COORDINATE INFO, JUST DUPLICATE THE PREVIOUS ONE
	    if(nextWaypoint[4]==1):
		reverseToLimitSwitch()
		deployParcel(servo) #deploy parcel function from servoControl.py
		
		#Checking if the ultrasonic flag is high, meaning we can use the ultrasonics to update the position
		#This will be the case when deploying in bins A and C but not B
		if(nextWaypoint[2]==1):
		    #check wall distance and update the robots X coordinate
		    xr = 10.375 + checkWallDistance(nextWaypoint[3]) #the distance from the center of the robot to the wall
		    location[0] =  (nextWaypoint[3])*(xr) + (1-nextWapoint[3])*(120-xr) #computes differently depending on the side that the wall is on
		    #y coordinate will always be the same when in contact with the North wall
		    location[1] = 120-11.265
		    #updating orientation
		    orientation = 180 #since we will now always be facing south

		#Removing the reverse and deploy waypoint from the list
		waypoints = np.delete(waypoints, 0) #deletes the 0th waypoint

	    else:
		#Compute trajectory of the waypoint
		distance, angle = computeTrajectory(location,orientation,nextWaypoint)
		#rotate heading
		rotate(angle)
		orientation = orientation + angle #updating orientation. Please double check my sign here
		time.sleep(1) #might not need
		#drive to waypoint
		translate(distance)

		#conditional location update
		#if the ultrasonic flag is high, meaning we can measure using ultrasonics
		#note that since we handle the reverse and deploy seperately, if we can measure here, it will always be on the south
		#side of the arena
		if(nextWaypoint[2]==1):
		    #side dependant on an element of the vector
		    #use the checkDistance function
		    xr = 10.375 + checkWallDistance(nextWaypoint[3]) #the distance from the center of the robot to the wall
		    location[0] =  (nextWaypoint[3])*(120-xr) + (1-nextWapoint[3])*(xr) #computes differently depending on the side that the wall is on
		    #y coordinate will always be the same when in contact with the South wall
		    location[1] = 11.265
		    #updating orientation
		    orientation = 0 #since we will now always be facing north
		
		#if not update the current location to be the waypoint
		else:
		    location = [nextWaypoint[0],nextWaypoint[1]]

		#remove waypoint from list
		waypoints = np.delete(waypoints, 0) #deletes the 0th waypoint
		"""

if __name__ == "__main__":
	while True:
		main()