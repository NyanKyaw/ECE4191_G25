import config
import cv2

def readimage(camera, detector):
    while camera.isOpened(): #while loop runs as long as webcam is running
        success, img = camera.read() #store label in 'img' variable
        value, points, qrcode = detector.detectAndDecode(img)
        value = str(value)
        print(value)

        #cv2.putText(img, value, (30,120), cv2.FONT_HERSHEY_SIMPLEX,2,(255,0,0))
        cv2.imshow('img', img)

        if value != "": #if QR code is detected and it's not empty, then extract points
            camera.release()
            if value == "A" or value == "A?":
                goal = config.Goals.BIN_A.value
            elif value == "B" or value == "B?":
                goal = config.Goals.BIN_B.value
            elif value == "C" or value == "C?":
                goal = config.Goals.BIN_C.value
            else: # if not one of the values we expect, keep reading 
                continue
            break
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows
    return goal

    # for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #     img = frame.array

    #     value = detector.detectAndDecode(img)

    #     if value != "":  # If QR code is detected and it's not empty, then extract points
    #         return value

    #     rawCapture.truncate(0)



