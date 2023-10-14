import cv2

# Initialize the camera, 0 is external camera, 1 is camera from iPhone
cap = cv2.VideoCapture(0)

# Check whether camera was opened correctly. If not, raise an error
if not cap.isOpened():
    raise IOError("Cannot open camera")

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    # If the frame was not captured correctly, break the loop
    if not ret:
        break

    # Display the image
    cv2.imshow('Camera Feed', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
