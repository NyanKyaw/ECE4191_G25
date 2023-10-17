# Script for testing basic navigation
# Integrated with limit switch
import RPi.GPIO as GPIO

import serial
import time
LIMIT_SWITCH = 23

def pinsetup():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LIMIT_SWITCH, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

def test_limit_switch():
	ls_state = GPIO.input(LIMIT_SWITCH)
	if (ls_state == GPIO.HIGH):
		print("LS High!")
	else:
		print("LS Low")


def forward(distance):
	#distance to time conversion
	# set_time = (4/53.014)*distance
	# print(set_time)
	
	ser = serial.Serial('/dev/ttyUSB0', 9600)
	time.sleep(2)
	
	val_str = "-0.5+0.5\n"
	ser.write(val_str.encode())
	time.sleep(6) # For Forward/Backward
	
	stop = f"{0}\n"
	ser.write(stop.encode())
	ser.close()

	# set_forward = "-0.5+0.5\n"
	# print(set_forward)
	# ser.write(set_forward.encode())
	# print("Encoded")
	# time.sleep(set_time)
	# pause = f"0\n"
	# ser.write(pause.encode())
	# ser.close()
	# print("Closed")
	



def main():
	pinsetup()
	try:
		forward(53)
		
	except Exception as e:
		print("Error")
		
	GPIO.cleanup()

if __name__ == "__main__":
	main()
	


