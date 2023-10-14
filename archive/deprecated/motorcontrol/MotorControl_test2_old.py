import RPi.GPIO as GPIO
import time
# Set the GPIO mode
GPIO.setmode(GPIO.BCM)
# Define your motor control pins
EN = 22   # replace with your actual pin number
ENB = 23  # replace with your actual pin number
PWM1 = 17 # replace with your actual pin number
PWM2 = 27 # replace with your actual pin number
# Set up the motor control pins zz
GPIO.setup(EN, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(PWM1, GPIO.OUT)
GPIO.setup(PWM2, GPIO.OUT)
# Create PWM object
p = GPIO.PWM(EN, 100)  # 100Hz frequency
try:
    while True:
        '''
        # Forward/coast operation
        GPIO.output(ENB, GPIO.LOW)
        GPIO.output(PWM1, GPIO.HIGH)
        GPIO.output(PWM2, GPIO.LOW)
        p.start(50)  # Start with 50% duty cycle
        time.sleep(2)  # run for 2 seconds
        p.stop()  # Stop PWM
        '''

        # Reverse/coast operation
        GPIO.output(ENB, GPIO.HIGH)
        GPIO.output(ENB, GPIO.LOW)
        GPIO.output(PWM1, GPIO.LOW)
        GPIO.output(PWM2, GPIO.HIGH)
        p.start(80)  # Start with 50% duty cycle
        time.sleep(2)  # run for 2 seconds
        p.stop()  # Stop PWM
        '''
        # Coast operation
        GPIO.output(ENB, GPIO.HIGH)
        GPIO.output(PWM1, GPIO.LOW)
        GPIO.output(PWM2, GPIO.LOW)
        time.sleep(2)  # coast for 2 seconds
        '''
except KeyboardInterrupt:
    # Stop the motor and clean up GPIO state
    p.stop()
    GPIO.cleanup()
