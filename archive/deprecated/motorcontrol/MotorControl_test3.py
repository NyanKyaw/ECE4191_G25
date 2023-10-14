import RPi.GPIO as GPIO
import time

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Define your motor control pins
EN = 22   # replace with your actual pin number
ENB = 23  # replace with your actual pin number
PWM1 = 17 # replace with your actual pin number
PWM2 = 27 # replace with your actual pin number

# Define your encoder pins
encoderA =   # replace with your actual pin number
encoderB = 25  # replace with your actual pin number

encoderPos = 0
lastTime = time.time()

# Set up the motor control pins
GPIO.setup(EN, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(PWM1, GPIO.OUT)
GPIO.setup(PWM2, GPIO.OUT)

# Set up the encoder pins
GPIO.setup(encoderA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoderB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Create PWM object
p = GPIO.PWM(EN, 100)  # 100Hz frequency

def encoder_callback(channel):
    print("Working")
    # global encoderPos, lastTime
    # if GPIO.input(encoderA) == GPIO.input(encoderB):
    #     encoderPos += 1
    #     direction = 'Forward'
    # else:
    #     encoderPos -= 1
    #     direction = 'Backward'
    
    # currentTime = time.time()
    # elapsedTime = currentTime - lastTime
    # speed = encoderPos / elapsedTime
    # print('Speed: {} Direction: {}'.format(speed, direction))
    
    # # Reset encoder position and time
    # encoderPos = 0
    # lastTime = currentTime

GPIO.add_event_detect(encoderA, GPIO.BOTH, callback=encoder_callback(encoderA)) # should only callback on one of the encoders right?
GPIO.add_event_detect(encoderB, GPIO.BOTH)#, callback=encoder_callback)

# try:
#     while True:

#         encoder_callback()
        
#         # Forward/coast operation
#         GPIO.output(ENB, GPIO.LOW)
#         GPIO.output(PWM1, GPIO.HIGH)
#         GPIO.output(PWM2, GPIO.LOW)
#         p.start(50)  # Start with 50% duty cycle
#         time.sleep(2)  # run for 2 seconds
#         p.stop()  # Stop PWM
        
        
#         # Reverse/coast operation
#         GPIO.output(ENB, GPIO.LOW)
#         GPIO.output(PWM1, GPIO.LOW)
#         GPIO.output(PWM2, GPIO.HIGH)
#         p.start(80)  # Start with 50% duty cycle
#         time.sleep(2)  # run for 2 seconds
#         p.stop()  # Stop PWM

#         '''
#         # Coast operation
#         GPIO.output(ENB, GPIO.HIGH)
#         GPIO.output(PWM1, GPIO.LOW)
#         GPIO.output(PWM2, GPIO.LOW)
#         time.sleep(2)  # coast for 2 seconds
#         '''
# except KeyboardInterrupt:
#     # Stop the motor and clean up GPIO state
#     p.stop()
#     GPIO.cleanup()
