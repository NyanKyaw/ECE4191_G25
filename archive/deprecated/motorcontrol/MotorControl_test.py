import RPi.GPIO as GPIO
import time

# Define the GPIO pin numbers
PWM1_PIN = 18
PWM2_PIN = 23

# Set up the GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM1_PIN, GPIO.OUT)
GPIO.setup(PWM2_PIN, GPIO.OUT)

# Set up the PWM frequencies
pwm1 = GPIO.PWM(PWM1_PIN, 100)
pwm2 = GPIO.PWM(PWM2_PIN, 100)

# Function to drive the motor forward
def forward(speed):
    pwm1.start(speed)
    pwm2.start(0)

# Function to drive the motor backward
def backward(speed):
    pwm1.start(0)
    pwm2.start(speed)

# Function to stop the motor
def stop():
    pwm1.start(0)
    pwm2.start(0)

for i in range(1000):
    # Test the motor
    print("Spinning motor forward")
    forward(50)  # Spin forward at half speed
    time.sleep(2)  # Continue for 2 seconds
    
    print("Stopping motor")
    stop()  # Stop
    time.sleep(1)  # Wait for 1 second
    
    print("Spinning motor backward")
    backward(50)  # Spin backward at half speed
    time.sleep(2)  # Continue for 2 seconds
    
    print("Stopping motor")
    stop()  # Stop

# Cleanup the GPIO pins
GPIO.cleanup()
