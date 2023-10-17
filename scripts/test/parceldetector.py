import config
import cv2

def readimage(camera, detector):
    while camera.isOpened(): #while loop runs as long as webcam is running
        success, img = camera.read() #store label in 'img' variable
        value, points, qrcode = detector.detectAndDecode(img)
        value = str(value)
        cv2.imshow('img', img)
        if value != "": #if QR code is detected and it's not empty, then extract points
            camera.release()
            if value == "Location A" or value == "Location A\n":
                goal = "A"
                print(goal)
            elif value == "Location B" or value == "Location B\n":
                goal = "B"
                print(goal)
            elif value == "Location C" or value == "Location C\n":
                goal = "C"
                print(goal)
            else: # if not one of the values we expect, keep reading 
                continue
             
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    camera.release()
    cv2.destroyAllWindows()
    return goal
    
def run_camera():
	detector, camera = config.camerasetup()

	goal = None

	while goal == None: # infinite loop
		goal = readimage(detector=detector, camera=camera) # this function has its own while loop that will run infinitely until qr is detected
			
	return goal

if __name__== "__main__":
	run_camera()


