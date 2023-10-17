from gpiozero import Servo
from time import sleep

def moveServo(start, end, delta):
	incMove = (end, start)/100.0
	incTime = delta / 100.0
	
	for x in range(101):
		position = start + x*incMove
		servo.value = position
		time.sleep(incTime)
		

def test_servo():
    servo = Servo(22)
    """
    servo = Servo(22, min_pulse_width = 0.0015, max_pulse_width = 0.0025, frame_width = 0.02)
    servo.value = 1
    time.sleep(2)
 
    servo.angle = 10
    time.sleep(1)
    servo.angle = 10
    time.sleep(1)
   
    while True:
    	moveServo(servo.min(),servo.max(),20)
    	moveServo(servo.min(),servo.max(),20)

   """
    #print('min')
    servo.min()
    sleep(3)
    print('max')
    servo.max()
    sleep(3)
    print('min')
    servo.min()
    sleep(3)
    print('at min')
    

if __name__ == "__main__":
	test_servo()
