from gpiozero import RotaryEncoder, OutputDevice, PWMOutputDevice
import time

# Constants
EN = 22
ENB = 23
PWM1 = 17
PWM2 = 27
ENCODER_PIN_A = 20
ENCODER_PIN_B = 21
MAX_STEPS = 1000000
GEAR_RATIO = 75
ENCODER_CPR = 48  # Counts per revolution of the encoder
WHEEL_DIAMETER = 0.1  # Wheel diameter in meters
STEPS_PER_ROTATION = GEAR_RATIO * ENCODER_CPR

# Define devices
en = PWMOutputDevice(EN)
enb = OutputDevice(ENB)
pwm1 = OutputDevice(PWM1)
pwm2 = OutputDevice(PWM2)
encoder = RotaryEncoder(ENCODER_PIN_A, ENCODER_PIN_B, max_steps=MAX_STEPS)

def calculate_speed(steps, time_interval):
    rotations = steps / STEPS_PER_ROTATION
    distance = rotations * WHEEL_DIAMETER * 3.14159
    speed = distance / time_interval
    return speed

def control_motor(direction, duty_cycle, duration):
    enb.off()  # Equivalent of GPIO.LOW
    if direction == 'forward':
        pwm1.on()  # Equivalent of GPIO.HIGH
        pwm2.off()  # Equivalent of GPIO.LOW
    else:  # Reverse
        pwm1.off()  # Equivalent of GPIO.LOW
        pwm2.on()  # Equivalent of GPIO.HIGH
    en.value = duty_cycle / 100  # Set duty cycle (0.0 <= duty_cycle <= 1.0)
    time.sleep(duration)
    en.off()  # Stop PWM

try:
    while True:
        for direction in ['forward', 'reverse']:
            initial_steps = encoder.steps
            control_motor(direction, 0.5 if direction == 'forward' else 0.8, 2)  # Duty cycle as fraction of 1
            final_steps = encoder.steps
            speed = calculate_speed(abs(final_steps - initial_steps), 2)
            print(f"{direction.capitalize()} Speed: {speed} m/s")

except KeyboardInterrupt:
    pass  # gpiozero handles cleanup on program exit
