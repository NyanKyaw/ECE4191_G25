def readimage(camera, detector, rawCapture):
    # cap = cv2.VideoCapture(1)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) #set resolution of camera
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720) #resolution of camera

    # detector = cv2.QRCodeDetector()

    # while cap.isOpened(): #while loop runs as long as webcam is running
    #     success, img = cap.read() #store label in 'img' variable
    #     value, points, qrcode = detector.detectAndDecode(img)

    #     if value != "": #if QR code is detected and it's not empty, then extract points
    #         cap.release()
    #         return value

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array

        value = detector.detectAndDecode(img)

        if value != "":  # If QR code is detected and it's not empty, then extract points
            return value

        rawCapture.truncate(0)



