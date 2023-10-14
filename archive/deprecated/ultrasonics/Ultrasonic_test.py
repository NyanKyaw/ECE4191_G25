import time
from gpiozero import DistanceSensor

# Initialize the ultrasonic sensor
sensor = DistanceSensor(echo = 27, trigger = 17)

while True:
    distance = sensor.distance * 100  # Convert distance to centimeters
    print(f'Distance: {distance} cm')
    time.sleep(1) # Wait for 1 second
