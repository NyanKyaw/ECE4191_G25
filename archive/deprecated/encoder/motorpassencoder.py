import RPi.GPIO as GPIO
import time

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor Pins (assuming you're using an H-bridge like the L298N)
IN1 = 17  # Change as per your connection
IN2 = 18  # Change as per your connection

# Encoder Pin
ENCODER_PIN = 23  # Change as per your connection

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize encoder ticks
encoder_ticks = 0

# Motor Control Functions
def forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

def backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

# Encoder Callback Function
def encoder_callback(channel):
    global encoder_ticks
    encoder_ticks += 1
    print(f"Encoder Ticks: {encoder_ticks}")

# Attach Interrupt for Encoder
GPIO.add_event_detect(ENCODER_PIN, GPIO.RISING, callback=encoder_callback)

try:
    # Test: Run motor forward for 5 seconds, then stop
    forward()
    time.sleep(5)
    stop()

except KeyboardInterrupt:
    print("Interrupted!")
    GPIO.cleanup()

finally:
    GPIO.cleanup()
