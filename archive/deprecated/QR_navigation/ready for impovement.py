import cv2
from pyzbar.pyzbar import decode

class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.origin = (x, y)

    def navigate_to(self, destination_x, destination_y):
        while self.x != destination_x or self.y != destination_y:
            if self.x < destination_x:
                self.x += 1
            elif self.x > destination_x:
                self.x -= 1

            if self.y < destination_y:
                self.y += 1
            elif self.y > destination_y:
                self.y -= 1

            print(f"Moved to {self.x}, {self.y}")
    
    def navigate_back(self):
        self.navigate_to(*self.origin)

# Initialize the camera
cap = cv2.VideoCapture(1)

# Check whether camera is open
if not cap.isOpened():
    raise IOError("Cannot open camera")

# Initialize the robot at (0, 0)
robot = Robot(0, 0)

while True:
    # Capture a frame from the camera
    _, frame = cap.read()

    # Decode QR codes from the frame
    codes = decode(frame)

    # If no codes are detected, skip the rest of the loop
    if not codes:
        continue
    
    # Otherwise, take the first detected QR code
    code = codes[0]

    print(f"Detected QR code: {code.data}")

    # If a QR code is deteced, navigate to a predefined location
    # In this example, we're assuming that the QR code data is a string like "3, 4"
    destination = code.data.decode("utf-8").split(",")
    if len(destination) != 2:
        print("Unexpected QR code data format.")
        continue

    try:
        destination_x = int(destination[0])
        destination_y = int(destination[1])
    except ValueError:
        print("Could not parse QR code data.")
        continue

    robot.navigate_to(destination_x, destination_y)
    robot.navigate_back()

    # Display the image
    cv2.imshow('frame', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera for others to use
cap.release()

# Destroy the windows so they won't stay on the screen
cv2.destroyAllWindows()
