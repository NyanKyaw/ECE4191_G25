from gpiozero import RotaryEncoder
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

# Creating encoder object using GPIO pins 24 and 25
encoder = RotaryEncoder(20, 21, max_steps=1000000)

# Create PWM object
p = GPIO.PWM(EN, 100)  # 100Hz frequency

# Wheel parameters (replace with your actual values)
steps_per_rotation = 75 * 48 # number of encoder steps per full rotation
wheel_diameter = 0.1  # wheel diameter in meters

def calculate_speed(steps, time_interval):
    # Calculate the distance traveled in the time interval
    rotations = steps / steps_per_rotation
    distance = rotations * wheel_diameter * 3.14159  # distance = circumference * number of rotations
    speed = distance / time_interval  # speed = distance / time
    return speed

try:
    while True:
        initial_steps = encoder.steps

        # Forward/coast operation
        GPIO.output(ENB, GPIO.LOW)
        GPIO.output(PWM1, GPIO.HIGH)
        GPIO.output(PWM2, GPIO.LOW)
        p.start(50)  # Start with 50% duty cycle
        time.sleep(2)  # run for 2 seconds
        p.stop()  # Stop PWM

        final_steps = encoder.steps
        speed = calculate_speed(final_steps - initial_steps, 2)
        print(f"Forward Speed: {speed} m/s")

        initial_steps = encoder.steps

        # Reverse/coast operation
        GPIO.output(ENB, GPIO.LOW)
        GPIO.output(PWM1, GPIO.LOW)
        GPIO.output(PWM2, GPIO.HIGH)
        p.start(80)  # Start with 80% duty cycle
        time.sleep(2)  # run for 2 seconds
        p.stop()  # Stop PWM

        final_steps = encoder.steps
        speed = calculate_speed(initial_steps - final_steps, 2)
        print(f"Reverse Speed: {speed} m/s")

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
