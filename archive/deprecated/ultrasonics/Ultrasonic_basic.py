import RPi.GPIO as GPIO
import time
#Ultrasonic Sensor No. 1
TRIG_1 = 17
ECHO_1 = 27

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_1, GPIO.OUT)
GPIO.setup(ECHO_1, GPIO.IN)

#Ultrasonic Sensor No. 2
ECHO_2 = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(ECHO_2, GPIO.IN)

#Ultrasonic Sensor No. 3
ECHO_3 = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(ECHO_3, GPIO.IN)

while True:
#Sensor 1
	GPIO.output(TRIG_1, False)
	time.sleep(0.1) #0.2
	GPIO.output(TRIG_1, True)
	time.sleep(0.2) #1
	GPIO.output(TRIG_1, False)
	while GPIO.input(ECHO_1)==0:
		pulse_start=time.time()
	while GPIO.input(ECHO_1)==1:
		pulse_end=time.time()
	pulse_duration=pulse_end-pulse_start
	distance_1=pulse_duration*17150
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
		pulse_start=time.time()
	while GPIO.input(ECHO_2)==1:
		pulse_end=time.time()
	pulse_duration=pulse_end-pulse_start
	distance_2=pulse_duration*17150
	print("US 2 Distance:", distance_2)
	if distance_2 <=15:
		print("Redirect")
	#time.sleep(0.5)
	
#Sensor 3	
	GPIO.output(TRIG_1, False)
	time.sleep(0.1)
	GPIO.output(TRIG_1, True)
	time.sleep(0.2)
	GPIO.output(TRIG_1, False)
	while GPIO.input(ECHO_3)==0:
		pulse_start=time.time()
	while GPIO.input(ECHO_3)==1:
		pulse_end=time.time()
	pulse_duration=pulse_end-pulse_start
	distance_3=pulse_duration*17150
	print("US 3 Distance:", distance_3)
	if distance_3 <=15:
		print("Redirect")
	#time.sleep(0.5)
	
#Limit switch
	LIM_SWITCH = 26
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LIM_SWITCH, GPIO.IN)	
	if GPIO.input(LIM_SWITCH)==1:
		print("At Wall")
	

	

