import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)

#Ultrasonic Sensor No. 1 - left
TRIG_1 = 27
ECHO_1 = 2
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_1, GPIO.OUT)
GPIO.setup(ECHO_1, GPIO.IN)

#Ultrasonic Sensor No. 2 - front left
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

while True:
	print("running")
	
#Sensor 1
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
	if distance_1 <=15:
		print("Redirect")
		
	
#Sensor 2	
	GPIO.output(TRIG_1, False)
	time.sleep(0.1)
	GPIO.output(TRIG_1, True)
	time.sleep(0.2)
	GPIO.output(TRIG_1, False)
	while GPIO.input(ECHO_2)==0:
		pulse_start_2=time.time()
	while GPIO.input(ECHO_2)==1:
		pulse_end_2=time.time()
	pulse_duration_2=pulse_end_2-pulse_start_2
	distance_2=pulse_duration_2*17150
	print("US 2 Distance:", distance_2)
	if distance_2 <=15:
		print("Redirect")
	#time.sleep(0.5)
	
	"""
	
#Sensor 3	
	GPIO.output(TRIG_1, False)
	time.sleep(0.1)
	GPIO.output(TRIG_1, True)
	time.sleep(0.2)
	GPIO.output(TRIG_1, False)
	while GPIO.input(ECHO_3)==0:
		print("stuck")
		pulse_start_3=time.time()
	while GPIO.input(ECHO_3)==1:
		print("high")
		pulse_end_3=time.time()
	pulse_duration_3=pulse_end_3-pulse_start_3
	distance_3=pulse_duration_3*17150
	print("US 3 Distance:", distance_3)
	if distance_3 <=15:
		print("Redirect")
	#time.sleep(0.5)
	

	"""
	
#Sensor 4	
	GPIO.output(TRIG_1, False)
	time.sleep(0.1)
	GPIO.output(TRIG_1, True)
	
	time.sleep(0.2)
	GPIO.output(TRIG_1, False)
	while GPIO.input(ECHO_4)==0:
		
		pulse_start=time.time()
	while GPIO.input(ECHO_4)==1:
		pulse_end=time.time()
	pulse_duration=pulse_end-pulse_start
	distance_4=pulse_duration*17150
	print("US 4 Distance:", distance_4)
	if distance_4 <=15:
		print("Redirect")
	#time.sleep(0.5)
	
	"""

#Limit switch
	LIM_SWITCH = 26
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LIM_SWITCH, GPIO.IN)	
	if GPIO.input(LIM_SWITCH)==1:
		print("At Wall")
		
"""
	

