import time
import numpy as np
import RPi.GPIO as GPIO
from gpiozero import RotaryEncoder

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()

# Assigning parameter values
ppr = 300.8  # Pulses Per Revolution of the encoder
tstop = 20  # Loop execution duration (s)
tsample = 0.02  # Sampling period for code execution (s)
tdisp = 0.5  # Sampling period for values display (s)

# Creating encoder object using GPIO pins 24 and 25
encoder = RotaryEncoder(24, 25, max_steps=0)

# Define your motor control pins
EN = 22   # replace with your actual pin number
ENB = 23  # replace with your actual pin number
PWM1 = 17 # replace with your actual pin number
PWM2 = 27 # replace with your actual pin number

# Set up the motor control pins
GPIO.setup(EN, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(PWM1, GPIO.OUT)
GPIO.setup(PWM2, GPIO.OUT)

# Create PWM object
p = GPIO.PWM(EN, 100)  # 100Hz frequency

# Initializing previous values and starting main clock
anglecurr = 0
tprev = 0
tcurr = 0
tstart = time.perf_counter()

# Execution loop that displays the current
# angular position of the encoder shaft
print('Running code for', tstop, 'seconds ...')
print('(Turn the encoder.)')
while tcurr <= tstop:
    # Pausing for `tsample` to give CPU time to process encoder signal
    time.sleep(tsample)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart
    # Getting angular position of the encoder
    # roughly every `tsample` seconds (deg.)
    anglecurr = 360 / ppr * encoder.steps
    # Printing angular position every `tdisp` seconds
    if (np.floor(tcurr/tdisp) - np.floor(tprev/tdisp)) == 1:
        print("Angle = {:0.0f} deg".format(anglecurr))
    # Updating previous values
    tprev = tcurr

    # Motor control
    try:
        # Forward/coast operation
        GPIO.output(ENB, GPIO.LOW)
        GPIO.output(PWM1, GPIO.HIGH)
        GPIO.output(PWM2, GPIO.LOW)
        p.start(50)  # Start with 50% duty cycle
        time.sleep(2)  # run for 2 seconds
        p.stop()  # Stop PWM
        
        # Reverse/coast operation
        GPIO.output(ENB, GPIO.LOW)
        GPIO.output(PWM1, GPIO.LOW)
        GPIO.output(PWM2, GPIO.HIGH)
        p.start(80)  # Start with 50% duty cycle
        time.sleep(2)  # run for 2 seconds
        p.stop()  # Stop PWM

    except KeyboardInterrupt:
        # Stop the motor and clean up GPIO state
        p.stop()
        GPIO.cleanup()

print('Done.')
# Releasing GPIO pins
encoder.close()
