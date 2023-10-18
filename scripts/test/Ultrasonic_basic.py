import RPi.GPIO as GPIO
import time

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



def ultrasonics():
	#initialise variables
	count = 0
	dist_1_sum = 0
	dist_2_sum = 0
	dist_3_sum = 0
	dist_4_sum = 0
	dist_5_sum = 0
	while True:
		
	#Sensor 1
		GPIO.output(TRIG_1, False)
		time.sleep(0.1) #0.2
		GPIO.output(TRIG_1, True)
		time.sleep(0.2) #1
		GPIO.output(TRIG_1, False)
		while GPIO.input(ECHO_1)==0:
			pulse_start_1=time.time()
			print(1.1)
		while GPIO.input(ECHO_1)==1:
			pulse_end_1 = time.time()
			print(1.2)
		pulse_duration_1=pulse_end_1-pulse_start_1
		distance_1=pulse_duration_1*17150
			
	#Sensor 2	
		GPIO.output(TRIG_1, False)
		time.sleep(0.1)
		GPIO.output(TRIG_1, True)
		time.sleep(0.2)
		GPIO.output(TRIG_1, False)
		while GPIO.input(ECHO_2)==0:
			pulse_start_2=time.time()
			print(2.1)
		while GPIO.input(ECHO_2)==1:
			pulse_end_2=time.time()
			print(2.2)
		pulse_duration_2=pulse_end_2-pulse_start_2
		distance_2=pulse_duration_2*17150	
		
	#Sensor 3	
		GPIO.output(TRIG_1, False)
		time.sleep(0.1)
		GPIO.output(TRIG_1, True)
		time.sleep(0.2)
		GPIO.output(TRIG_1, False)
		while GPIO.input(ECHO_3)==0:
			pulse_start_3=time.time()
			print(3.1)
		while GPIO.input(ECHO_3)==1:
			pulse_end_3=time.time()
			print(3.2)
		pulse_duration_3=pulse_end_3-pulse_start_3
		distance_3=pulse_duration_3*17150
			
	#Sensor 4	
		GPIO.output(TRIG_1, False)
		time.sleep(0.1)
		GPIO.output(TRIG_1, True)
		time.sleep(0.2)
		GPIO.output(TRIG_1, False)
		while GPIO.input(ECHO_4)==0:
			pulse_start=time.time()
			print(4.1)
		while GPIO.input(ECHO_4)==1:
			pulse_end=time.time()
			print(4.2)
		pulse_duration=pulse_end-pulse_start
		distance_4=pulse_duration*17150
	
	#Sensor 5	
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
		distance_5=pulse_duration*17150
		
	
		for x in range(32000):
			count+=1
			dist_1_sum += distance_1
			dist_2_sum += distance_2
			dist_3_sum += distance_3
			dist_4_sum += distance_4
			dist_5_sum += distance_5

		if count == 32000:
			dist_1_average = dist_1_sum/32000
			dist_2_average = dist_2_sum/32000
			dist_3_average = dist_3_sum/32000
			dist_4_average = dist_4_sum/32000
			dist_5_average = dist_5_sum/32000
			
			# Reset values
			count = 0
			dist_1_sum=0
			dist_2_sum=0
			dist_3_sum=0
			dist_4_sum=0
			dist_5_sum=0


			return dist_1_average, dist_2_average, dist_3_average, dist_4_average, dist_5_average

if __name__ == "__main__":
	dist_1_average, dist_2_average, dist_3_average, dist_4_average, dist_5_average = ultrasonics()
	print("Dist1: ")
	print(dist_1_average)
	print("\nDist2: ")
	print(dist_2_average)
	print("\nDist3: ")
	print(dist_3_average)
	print("Dist4: ")
	print(dist_4_average)
	print("\nDist5: ")
	print(dist_5_average)
	

