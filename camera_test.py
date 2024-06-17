import cv2
from picamera2 import Picamera2
import numpy as np
import time

def initialize_camera(**kwargs):
    frame_width = kwargs.get("width", 320)
    frame_height = kwargs.get("height", 240)
    frame_rate = kwargs.get("frame_rate", 30)
    
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (frame_width, frame_height)}, controls={"FrameRate": frame_rate})
    picam2.configure(config)
    return picam2

def camera_loop(camera, classifier):
    while True:
        frame = camera.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = classifier.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)

        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        cv2.imshow("Frame", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

if __name__ == "__main__":
    camera = initialize_camera(width=320, height=240, frame_rate=30)
    camera.start()

    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    time.sleep(2)

    camera_loop(camera, face_cascade)