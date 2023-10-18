from ultrasonic_split import ultrasonic_1, ultrasonic_2, ultrasonic_3, ultrasonic_4, ultrasonic_5
import time
import serial
lw_velocity_target = 0.5
rw_velocity_target = 0.5
Kp = 0.045
error = 0
set_time = 8

def p_wall_controller(set_time): # forward
	ser = serial.Serial('/dev/ttyUSB0', 9600)
	time.sleep(2)

	start_time = time.time()
	current_time = time.time()

	while current_time-start_time < set_time: 
		predicted = 13
		u_1 = ultrasonic_1() #need indexing and if condition to choose right ultrasonic
		u_2 = ultrasonic_1()
		# if going forward
		error = (u_1-u_2)
		print(f"Ultrasonic 1 {u_1}\nUltrasonic 2 {u_2}")
		# if right sensor is triggered
		rw_velocity_out = rw_velocity_target - Kp*error #if error is pos increase rw
		lw_velocity_out = lw_velocity_target + Kp*error # if error is neg increase lw

		# safety net
		if abs(rw_velocity_out)>1:
			rw_velocity_out = 1 *(rw_velocity_out/abs(rw_velocity_out)) # fraction sets sign
		
		if abs(lw_velocity_out)>1:
			lw_velocity_out = 1 *(lw_velocity_out/abs(lw_velocity_out)) # fraction sets sign

		print(f"LW OUT {lw_velocity_out}\nRW OUT {rw_velocity_out}")
		if (rw_velocity_out)>0:
			if (lw_velocity_out)>0:
				val_str = f"-{rw_velocity_out:.1f}+{lw_velocity_out:.1f}\n"
			elif (lw_velocity_out)<0:
				val_str = f"-{rw_velocity_out:.1f}-{abs(lw_velocity_out):.1f}\n"
		elif rw_velocity_out<0:
			if (lw_velocity_out)>0:
				val_str = f"+{abs(rw_velocity_out):.1f}+{lw_velocity_out:.1f}\n"
			elif (lw_velocity_out)>0:
				val_str = f"+{abs(rw_velocity_out):.1f}-{abs(lw_velocity_out):.1f}\n"
		
		print(val_str)
		ser.write(val_str.encode())
		#time.sleep(0.5)

		current_time = time.time()
		
	stop = f"{0}\n"
	ser.write(stop.encode())
	ser.close()
	
if __name__ == "__main__":
	p_wall_controller(set_time)


