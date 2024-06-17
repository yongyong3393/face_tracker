import RPi.GPIO as GPIO
from picamera2 import Picamera2
import cv2
import time
import numpy as np

class Servo:
    min_duty = 3
    max_duty = 12

    def __init__(self, pin_number):
        self.pin_number = pin_number
        GPIO.setup(self.pin_number, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin_number, 50)

    def start(self):
        self.pwm.start(0)

    def move(self, theta):
        if theta < 0:
            theta = 0
        elif theta > 180:
            theta = 180
        print('Moving servo', self.pin_number, 'to', theta)
        duty = Servo.min_duty + (Servo.max_duty - Servo.min_duty) * theta / 180
        self.pwm.ChangeDutyCycle(duty)

    def stop(self):
        self.pwm.stop()

class Camera:
    def __init__(self, **kwargs):
        self.width = kwargs.get("width", 320)
        self.height = kwargs.get("height", 240)
        self.frame_rate = kwargs.get("frame_rate", 30)

        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (self.width, self.height)}, controls={"FrameRate": self.frame_rate})
        self.picam2.configure(config)

        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')      

    def start(self):
        self.picam2.start()

    def get_face_coordinates(self):
        self.frame = self.picam2.capture_array()
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        self.faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)
        return self.faces
    
    def draw_faces(self):
        for (x, y, w, h) in self.faces:
            cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.imshow("Frame", self.frame)

def move_servo(x, y):
    if np.isnan(x) or np.isnan(y):
        print("yes!!")
        return
    else:
        err_x = (x - frame_width / 2) / frame_width
        err_y = (frame_height / 2 - y) / frame_height
    
    Kpx = 30
    Kpy = 30

    # servo1.move(90 + Kpx * err_x)
    servo2.move(30 + Kpy * err_y)

# Constants
servo_pin_1 = 32
servo_pin_2 = 33

frame_width = 640
frame_height = 480
frame_rate = 30

if __name__ == '__main__':
    # Initialize the servo and camera
    GPIO.setmode(GPIO.BOARD)
    servo1 = Servo(servo_pin_1)
    servo2 = Servo(servo_pin_2)
    camera = Camera(width=frame_width, height=frame_height, frame_rate=frame_rate)
    time.sleep(2)

    # Start the servo and camera
    servo1.start()
    servo2.start()
    camera.start()
    time.sleep(1)

    # main loop
    while True:
        # Get the face coordinates
        faces = camera.get_face_coordinates()
        camera.draw_faces()

        # Move the servo
        max_area = 0
        x_center, y_center = float('nan'), float('nan')
        for face in faces:
            x, y, w, h = face
            if w * h > max_area:
                max_area = w * h
                x_center = x + w / 2
                y_center = y + h / 2
        print(x_center, y_center)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        move_servo(x_center, y_center)
        time.sleep(0.1)
    
    # Clean up
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()