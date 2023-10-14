def readimage(camera, detector):
    while camera.isOpened(): #while loop runs as long as webcam is running
        success, img = camera.read() #store label in 'img' variable
        value, points, qrcode = detector.detectAndDecode(img)

        if value != "": #if QR code is detected and it's not empty, then extract points
            # camera.release()
            return value

    # for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #     img = frame.array

    #     value = detector.detectAndDecode(img)

    #     if value != "":  # If QR code is detected and it's not empty, then extract points
    #         return value

    #     rawCapture.truncate(0)



