import config
import cv2

# set variables
# distance from wall
dist_from_wall = 14.5

def readimage(camera, detector):
    goal = None
    while camera.isOpened(): #while loop runs as long as webcam is running
        success, img = camera.read() #store label in 'img' variable
        value, points, qrcode = detector.detectAndDecode(img)
        value = str(value)
        cv2.imshow('img', img)
        if value != "": #if QR code is detected and it's not empty, then extract points
            camera.release()
            if value == "Location A" or value == "Location A\n":
                goal = [[23.2,94,0,0,0,0],[dist_from_wall,100,0,0,0,0],[dist_from_wall,120,1,1,1], [dist_from_wall,100,0,0,1], [23.2,94,0,0,0], [23.2, 12.02, 1, 0, 1, 1]]
                #Goal formatting here is:
                #[X,Y,UltrasonicFlag,Side,ReverseFlag. DeployFlag]
                #X and Y are waypoint coordinates
                #UltrasonicFlag is 1 if ultrasonics can be used at that waypoint to check location
                #Side is the side to take a measurement from if the ultrasonics can be used. 0 is left, 1 is right
                #Reverse flag is 1 if a reverse until limit switch 
                #Deploy flag is 1 is lever needs to be deployed at waypoint

                #NOTE: If the reverse and deploy is to be initiated, and DeployFlag is high, all other properties of that
                #waypoint will be ignored, so duplicate the previous waypoint
                print(goal)
            elif value == "Location B" or value == "Location B\n":
                #goal = #some goal matrix
                print(goal)
            elif value == "Location C" or value == "Location C\n":
               # goal = #some goal matrix
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
