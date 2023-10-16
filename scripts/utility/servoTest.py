from gpiozero import Servo
from time import sleep

servo = Servo(4)

"""print("move to min\n")
servo.min()
sleep(5)
#servo.mid()
#sleep(5)
print("move to max\n")
servo.max()
sleep(5)
print("move to min\n")
servo.min()
sleep(5)
print("move to middle")
servo.value = 1
sleep(5)"""

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


