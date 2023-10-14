### Define Global Configs ###
# Can also just use an enum class #

### MOTOR PINS - Old Driver ###
# Define control pins for left motor
#ML_EN = 22 # Could just keep high
#ML_PWM1 = 17
#ML_PWM2 = 27

# Define encoder pins for left motor
#ML_ENCA = 20
#ML_ENCB = 21

# Define control pins for right motor
#M2_EN = 23
#MR_PWM1 = 22
#MR_PWM2 = 23

# Define encoder pins for right motor
#MR_ENCA = 24
#MR_ENCB = 25

### MOTOR PINS - L298N ###
# Left
Left_IN1 = 17
Left_IN2 = 27
Left_PWM = 22

# Right
Right_IN3 = 24
Right_IN4 = 25
Right_PWM = 23

# Encoder Pins
Left_ENCA = 5
Left_ENCB = 6

Right_ENCA = 16
Right_ENCB = 26
