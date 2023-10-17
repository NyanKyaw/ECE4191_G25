import serial
import time


try: 
	ser = serial.Serial('/dev/ttyUSB0', 9600)
	time.sleep(2)

    # Change to directions 
    # Forward val = +0.5-0.5 - 4 seconds - Dist = 
    # Backward val = -0.5+0.5
    # Left val = -0.5-0.5
    # Right val = +0.5+0.5
	#val = input("Enter the directions: ")
	#print(val)
	val_str = "+0.5-0.5\n"
	ser.write(val_str.encode())
	time.sleep(6) # For Forward/Backward
	time.sleep(0.502) # for turning circle
	
	stop = f"{0}\n"
	ser.write(stop.encode())
	ser.close()
	print("Tried")
	
except Exception as e:
	print("Error")
