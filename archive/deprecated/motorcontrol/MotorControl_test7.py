from gpiozero import Robot, RotaryEncoder
import time

# Define motor pins
left_motor_pins = (17, 27)  # (forward, backward)
right_motor_pins = (22, 23)  # (forward, backward)

# Define encoder pins
left_encoder_pins = (20, 21)
right_encoder_pins = (24, 25)

# Create Robot and Encoder objects
robot = Robot(left=left_motor_pins, right=right_motor_pins, pwm=True)
left_encoder = RotaryEncoder(*left_encoder_pins, max_steps=1000000)
right_encoder = RotaryEncoder(*right_encoder_pins, max_steps=1000000)

# Define a function to compute speed from encoder steps
def compute_speed(steps, time_interval):
    """Compute the speed given the number of steps and the time interval."""
    # Assuming 1 step corresponds to 1 rotation
    distance_per_step = 0.1 * 3.14159  # Wheel circumference = diameter * pi
    distance = steps * distance_per_step
    speed = distance / time_interval
    return speed

# Drive the robot forward for 2 seconds and compute speed
robot.forward()
time.sleep(20)
robot.stop()

# Compute speed of left wheel
initial_steps_left = left_encoder.steps
final_steps_left = left_encoder.steps
speed_left = compute_speed(abs(final_steps_left - initial_steps_left), 2)
print(f'initial_steps_left: {initial_steps_left}')
print(f'final_steps_left: {final_steps_left}')
print(f"Left Wheel Speed: {speed_left} m/s")

# Compute speed of right wheel
initial_steps_right = right_encoder.steps
final_steps_right = right_encoder.steps
speed_right = compute_speed(abs(final_steps_right - initial_steps_right), 2)
print(f"Right Wheel Speed: {speed_right} m/s")
