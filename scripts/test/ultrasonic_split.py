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
smoothing_factor = 10
def ultrasonic_1():
	dist_1_sum = 0
	count = 0
	list_1 = []
	for x in range(smoothing_factor):
		GPIO.output(TRIG_1, False)
		time.sleep(0.1)
		GPIO.output(TRIG_1, True)
		time.sleep(0.1)
		GPIO.output(TRIG_1, False)
		while GPIO.input(ECHO_1)==0:
			pulse_start=time.time()
			#print(1.1)
		while GPIO.input(ECHO_1)==1:
			pulse_end=time.time()
			#print(1.1)
		pulse_duration=pulse_end-pulse_start
		distance_1=pulse_duration*17150

		count+=1
		dist_1_sum += distance_1
		list_1.append(distance_1)

		if count == smoothing_factor:
			dist_1_average = dist_1_sum/smoothing_factor

			# Reset values
			count = 0
			return dist_1_average

def ultrasonic_2():
	dist_2_sum = 0
	count = 0
	list_2 = []
	for x in range(smoothing_factor):
		GPIO.output(TRIG_1, False)
		time.sleep(0.1)
		GPIO.output(TRIG_1, True)
		time.sleep(0.2)
		GPIO.output(TRIG_1, False)
		while GPIO.input(ECHO_2)==0:
			pulse_start=time.time()
			#print(2.1)
		while GPIO.input(ECHO_2)==1:
			pulse_end=time.time()
			#print(2.2)
		pulse_duration=pulse_end-pulse_start
		distance_2=pulse_duration*17150

		count+=1
		dist_2_sum += distance_2
		list_2.append(distance_2)

		if count == smoothing_factor:
			dist_2_average = dist_2_sum/smoothing_factor
			# Reset values
			count = 0
			return dist_2_average


def ultrasonic_3():
	dist_3_sum = 0
	count = 0
	list_3 = []
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
		distance_3=pulse_duration*17150

		count+=1
		dist_3_sum += distance_3
		list_3.append(distance_3)

		if count == smoothing_factor:
			dist_3_average = dist_3_sum/smoothing_factor
			# Reset values
			count = 0
			return dist_3_average


smoothing_factor = 10

def ultrasonic_4():
	dist_4_sum = 0
	count = 0
	list_4 = []
	for x in range(smoothing_factor):
		GPIO.output(TRIG_1, False)
		time.sleep(0.1)
		GPIO.output(TRIG_1, True)
		time.sleep(0.2)
		GPIO.output(TRIG_1, False)
		while GPIO.input(ECHO_4)==0:
			pulse_start=time.time()
			#print(4.1)
		while GPIO.input(ECHO_4)==1:
			pulse_end=time.time()
			#print(4.2)
		pulse_duration=pulse_end-pulse_start
		distance_4=pulse_duration*17150

		count+=1
		dist_4_sum += distance_4
		list_4.append(distance_4)

		if count == smoothing_factor:
			dist_4_average = dist_4_sum/smoothing_factor
			# Reset values
			count = 0
			return dist_4_average
	
def ultrasonic_5():
	dist_5_sum = 0
	count = 0
	while True:	
		for x in range(smoothing_factor):
			GPIO.output(TRIG_1, False)
			time.sleep(0.1)
			GPIO.output(TRIG_1, True)
			time.sleep(0.2)
			GPIO.output(TRIG_1, False)
			while GPIO.input(ECHO_5)==0:
				pulse_start=time.time()
				#print(5.1)
			while GPIO.input(ECHO_5)==1:
				pulse_end=time.time()
				#print(5.2)
			pulse_duration=pulse_end-pulse_start
			distance_5=pulse_duration*17150

		
			count+=1
			dist_5_sum += distance_5

			if count == smoothing_factor:
				dist_5_average = dist_5_sum/smoothing_factor
			
				# Reset values
				count = 0
				dist_5_sum=0
				return dist_5_average

if __name__ == "__main__":
	print("Running")

	print(f"US 1: {ultrasonic_1()}\n")
	print(f"US 2: {ultrasonic_2()}\n")
	print(f"US 3: {ultrasonic_3()}\n")
	print(f"US 4: {ultrasonic_4()}\n")
	print(f"US 5: {ultrasonic_5()}\n")

	

