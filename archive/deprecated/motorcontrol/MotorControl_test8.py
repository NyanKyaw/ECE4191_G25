from gpiozero import Robot, DistanceSensor
import time

# Define motor and sensor pins
left_motor_pins = (17, 27)  # (forward, backward)
right_motor_pins = (22, 23)  # (forward, backward)
echo_pin = 24
trigger_pin = 25

# Create Robot and DistanceSensor objects
robot = Robot(left=left_motor_pins, right=right_motor_pins)
sensor = DistanceSensor(echo=echo_pin, trigger=trigger_pin)

while True:
    robot.forward()
    # If the distance to the nearest object is less than 1 meter, stop
    if sensor.distance * 100 < 100:
        robot.stop()
        break
    time.sleep(0.1)
