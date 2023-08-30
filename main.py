### We will implement all control scripts in this file, and it will be what is run ###

import RPi.GPIO as GPIO
import config
from robot import CustomRobot

GPIO.setmode(GPIO.BCM)

### Set MOTOR Pins ###
#left_motor_pins = (config.ML_PWM1, config.ML_PWM2)
left_motor_pins = (config.Left_IN1, config.Left_IN2, config.Left_PWM)
#right_motor_pins = (config.MR_PWM1, config.MR_PWM2)
right_motor_pins = (config.Right_IN3, config.Right_IN4, config.Right_PWM)

#left_encoder_pins = (config.ML_ENCA, config.ML_ENCB)
#right_encoder_pins = (config.MR_ENCA, config.MR_ENCB)

### Robot Params ###
distance_per_step = 0.1 * 3.14159  # Wheel circumference = diameter * pi

# Setup pins
GPIO.setup(config.Left_IN1, GPIO.OUT)
GPIO.setup(config.Left_IN2, GPIO.OUT)
GPIO.setup(config.Right_IN3, GPIO.OUT)
GPIO.setup(config.Right_IN4, GPIO.OUT)
GPIO.setup(config.Left_PWM, GPIO.OUT)
GPIO.setup(config.Right_PWM, GPIO.OUT)

# Set up the PWM frequencies
pwm1 = GPIO.PWM(config.Left_PWM, 100)
pwm2 = GPIO.PWM(config.Right_PWM, 100)


def main():
    customRobot = CustomRobot(left_motor_pins, right_motor_pins, distance_per_step)
    
    pwm1.start(100)
    pwm2.start(100)
    
    try:
    	while True:
            customRobot.drive_robot(velocity=[1,0], duration=2)
            customRobot.drive_robot(velocity=[-1,0], duration=2)
            customRobot.drive_robot(velocity=[0,1], duration=2)
            customRobot.drive_robot(velocity=[0,-1], duration=2)
    except KeyboardInterrupt:
    	pwm1.stop()
    	pwm2.stop()
    	GPIO.cleanup()
    	
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
    
if __name__ == "__main__":
    main()
