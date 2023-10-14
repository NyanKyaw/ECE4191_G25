import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Define control pins for motor1
EN1 = 22
IN1 = 17
IN2 = 27

# Define control pins for motor2
EN2 = 23
IN3 = 24
IN4 = 25

# Setup the motor control pins
GPIO.setup(EN1, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(EN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Create PWM objects
pwm1 = GPIO.PWM(EN1, 100)  # motor1 with 100Hz frequency
pwm2 = GPIO.PWM(EN2, 100)  # motor2 with 100Hz frequency

try:
    while True:
        # Forward operation
        pwm1.start(100)  # Start motor1 with 100% duty cycle
        pwm2.start(100)  # Start motor2 with 100% duty cycle
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        time.sleep(2)  # run for 2 seconds
        pwm1.stop()  # Stop PWM motor1
        pwm2.stop()  # Stop PWM motor2

        # Reverse operation
        pwm1.start(100)  # Start motor1 with 100% duty cycle
        pwm2.start(100)  # Start motor2 with 100% duty cycle
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        time.sleep(2)  # run for 2 seconds
        pwm1.stop()  # Stop PWM motor1
        pwm2.stop()  # Stop PWM motor2

        # Rotate operation (assume clockwise)
        pwm1.start(100)  # Start motor1 with 100% duty cycle
        pwm2.start(100)  # Start motor2 with 100% duty cycle
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        time.sleep(2)  # run for 2 seconds
        pwm1.stop()  # Stop PWM motor1
        pwm2.stop()  # Stop PWM motor2

except KeyboardInterrupt:
    # Stop the motor and clean up GPIO state
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
