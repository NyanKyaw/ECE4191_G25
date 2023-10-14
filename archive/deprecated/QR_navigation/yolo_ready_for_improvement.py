import cv2
import numpy as np
from yolov5 import YOLOv5

class Robot:
    def __init__(self, yolo, target_area, cap):
        self.yolo = yolo
        self.target_area = target_area
        self.cap = cap

    def navigate_to(self):
        while True:
            # Capture a frame from the camera
            ret, frame = self.cap.read()

            # Get the width of the frame
            frame_width = frame.shape[1]
            
            # Use YOLO to detect QR codes in the frame
            # Will define a funtion called detect()
            results = self.yolo.detect(frame)

            # If no codes are detected, continue the loop

            # results.xyxy is the bounding box coordinates,
            if len(results.xyxy[0]) == 0:
                continue

            # Otherwise, take the detected QR code
            code = results.xyxy[0][0]

            # Calculate the center of the QR code and its area
            left, top, right, bottom = code[:4]
            qr_center_x = (right + left) / 2
            qr_area = (right - left) * (bottom - top)

            # If QR code's center_x is in the left half of the image, move left
            if qr_center_x < frame_width / 2:
                # Replace with actual command to move robot left
                print("Moving left")

            # If QR code's center_x is in the right half of the image, move right
            elif qr_center_x > frame_width / 2:
                # Replace with actual command to move robot right
                print("Moving right")
                
            # If QR code's area is smaller than the target area, move forward
            if qr_area < self.target_area:
                # Replace with actual command to move robot forward
                print("Moving forward")
            else:
                print("Reached the QR code, stopping movement.")
                break
            
# Initialize camera
cap = cv2.VideoCapture(1)

# Initialize YOLO
yolo = YOLOv5("path_to_yolov5_weights")

# Initialize the robot with the trained YOLO model and a target area of 10000
robot = Robot(yolo, 10000, cap)

robot.navigate_to()

cap.release()
cv2.destoryAllWindows()
