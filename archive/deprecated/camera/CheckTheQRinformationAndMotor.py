import cv2
from pyzbar.pyzbar import decode

class Robot:
    def __init__(self, x, y, cap):
        self.x = x
        self.y = y
        self.cap = cap
        self.target_area = 150000

    def navigate_to(self):
        qr_area = 0
        tolerance = 0.4  # 10% tolerance
        while qr_area < self.target_area:
            _, frame = self.cap.read()
            codes = decode(frame)

            if not codes:
                print("No QR code detected")
                continue

            for code in codes:
                left, top, width, height = code.rect
                qr_center_x = int(left + width / 2)
                qr_center_y = int(top + height / 2)
                qr_area = width * height

                print(f"QR Code Information:")
                print(f"Data: {code.data.decode('utf-8')}")
                print(f"Type: {code.type}")
                print(f"Coordinates: {code.polygon}")
                print(f"Area: {qr_area}")  

                cv2.rectangle(frame, (left, top), (left + width, top + height), (0, 255, 0), 3)
                cv2.circle(frame, (qr_center_x, qr_center_y), radius=5, color=(0, 0, 255), thickness=-1)

            cv2.imshow('QR Code Detection', frame)

            frame_center_x = frame.shape[1] / 2
            left_limit = frame_center_x - frame.shape[1] * tolerance
            right_limit = frame_center_x + frame.shape[1] * tolerance

            # If QR code's center_x is in the left half of the image and outside the tolerance range, move left
            if qr_center_x < left_limit:
                print("Moving left")

            # If QR code's center_x is in the right half of the image and outside the tolerance range, move right
            elif qr_center_x > right_limit:
                print("Moving right")

            if qr_area < self.target_area:
                print("Moving forward")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        print("Reached the QR code, stopping movement.")

# Initialize the camera
cap = cv2.VideoCapture(0)

# Check whether camera is open
if not cap.isOpened():
    raise IOError("Cannot open camera")

# Initialize the robot at (0, 0)
robot = Robot(0, 0, cap)

robot.navigate_to()

# Release the camera for others to use
cap.release()

# Destroy the windows so they won't stay on the screen
cv2.destroyAllWindows()
