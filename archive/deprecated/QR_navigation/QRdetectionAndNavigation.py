import cv2
from pyzbar.pyzbar import decode
# Import the motor control file
from archive.deprecated.motorcontrol.MotorControl import forward, backward, stop

class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def navigate_to(self, destination_x, destination_y):
        while self.x != destination_x or self.y != destination_y:
            if self.x < destination_x:
                # move forward at 50% speed
                forward(50)
                self.x += 1
            elif self.x > destination_x:
                # move backward at 50% speed
                backward(50)
                self.x -= 1

            if self.y < destination_y:
                # move forward at 50% speed
                forward(50)
                self.y += 1
            elif self.y > destination_y:
                # move backward at 50% speed
                backward(50)
                self.y -= 1

            print(f"Moved to {self.x}, {self.y}")

# initialize the camera
cap = cv2.VideoCapture(1)

# check whether camera is open
if not cap.isOpened():
    raise IOError("Cannot open camera")

# initialize the robot at (0, 0)
robot = Robot(0, 0)

while True:
    # capture a frame from the camera
    _, frame = cap.read()

    # decode QR codes from the frame
    codes = decode(frame)

    # if no codes are detected, raise an error
    if not codes:
        raise ValueError("No QR code detected")
    
    # otherwise, take the detected QR code
    code = codes[0]

    print(f"Detected QR code: {code.data}")

    # if a QR code is deteced, navigate to a predefined location
    # in this example, we're assuming that the QR code data is a string like "3, 4"
    destination = code.data.decode("utf-8").split(",")
    destination_x = int(destination[0])
    destination_y = int(destination[1])
    robot.navigate_to(destination_x, destination_y)

    # display the image
    cv2.imshow('frame', frame)

    # break the loop if 'q' is pressed
    if cv2.waitKey(1) and 0xff == ord('q'):
        break

# release the camera for others to use
cap.release()
# destory the windows so they won't stay on the screen
cv2.destoryAllWindows()

# stop the motors
