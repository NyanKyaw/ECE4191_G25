import cv2
import numpy as np

class Robot:
    def __init__(self, x, y, cap, net, layer_names):
        self.x = x
        self.y = y
        self.cap = cap
        self.net = net
        self.layer_names = layer_names

    def navigate_to(self):
        while True:
            # Capture a frame from the camera
            _, frame = self.cap.read()

            height, width, _ = frame.shape
            
            # Preprocess the image for YOLO
            blob = cv2.dnn.blobFromImage(frame, 1/255, (416, 416), (0, 0, 0), swapRB=True, crop=False)
            self.net.setInput(blob)

            # Perform forward pass through the network
            layer_outputs = self.net.forward(self.layer_names)
            
            # Loop over each of the detections
            for output in layer_outputs:
                for detection in output:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]

                    # If confidence > 0.5 and class_id is 0 (assuming that class 0 is a person, obstacle)
                    if confidence > 0.5 and class_id == 0:
                        # Object detected, avoid it
                        box = detection[0:4] * np.array([width, height, width, height])
                        (centerX, centerY, objectWidth, objectHeight) = box.astype("int")
                        
                        if centerX < width / 2:
                            # Object is on the left, move right
                            print("Moving right")
                        else:
                            # Object is on the right, move left
                            print("Moving left")
            # Replace with actual command to move robot forward
            print("Moving forward")

# Load YOLO
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# Initialize the camera
cap = cv2.VideoCapture(0)

# Check whether camera is open
if not cap.isOpened():
    raise IOError("Cannot open camera")

# Initialize the robot at (0, 0)
robot = Robot(0, 0, cap, net, output_layers)

robot.navigate_to()

# Release the camera for others to use
cap.release()
# Destroy the windows so they won't stay on the screen
cv2.destroyAllWindows()
