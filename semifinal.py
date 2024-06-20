# common library
import pigpio
import time
import numpy as np

# for vision
from picamera2 import Picamera2
import cv2

# for ir remote control
import subprocess
import threading

class Servo:
    # Pulse width between 500us to 2500us
    min_pulsewidth = 500
    max_pulsewidth = 2500
    offset = 150

    def __init__(self, pin_number):
        self.pin_number = pin_number
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise IOError("Failed to connect to pigpio daemon")
        self.angle = 0
        self.running = False
        self.target_direction = None
        self.thread = None

    def move_continuous(self, direction):
        if not self.running:
            self.running = True
            self.target_direction = direction
            print(f"Servo {self.pin_number} is moving continuously in {direction} direction")
            self.thread = threading.Thread(target=self._move_servo_continuous)
            self.thread.start()
    
    def _move_servo_continuous(self):
        while self.running:
            if self.target_direction == "increase":
                if self.angle < 180:
                    self.angle += 1
            elif self.target_direction == "decrease":
                if self.angle > 0:
                    self.angle -= 1

            pulsewidth = Servo.offset + Servo.min_pulsewidth + (self.angle / 180.0) * (Servo.max_pulsewidth - Servo.min_pulsewidth)
            if pulsewidth > 2500:
                pulsewidth = 2500
            self.pi.set_servo_pulsewidth(self.pin_number, pulsewidth)

            time.sleep(0.03)

    def move(self, angle):
        self.angle += angle
        if self.angle < 0:
            self.angle = 0
        elif self.angle > 180:
            self.angle = 180
        print(f'Moving servo {self.pin_number} to {self.angle}')

        pulsewidth = Servo.offset + Servo.min_pulsewidth + (self.angle / 180.0) * (Servo.max_pulsewidth - Servo.min_pulsewidth)
        if pulsewidth > 2500:
            pulsewidth = 2500
        self.pi.set_servo_pulsewidth(self.pin_number, pulsewidth)

    def stop(self):
        self.running = False
        self.pi.set_servo_pulsewidth(self.pin_number, 0)

class Camera:
    def __init__(self, **kwargs):
        self.width = kwargs.get("width", 320)
        self.height = kwargs.get("height", 240)
        self.frame_rate = kwargs.get("frame_rate", 30)

        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (self.width, self.height)}, controls={"FrameRate": self.frame_rate})
        self.picam2.configure(config)
        self.faces = None

        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    def start(self):
        self.picam2.start()
    
    def stop(self):
        self.picam2.stop()

    def get_face_coordinates(self):
        self.frame = self.picam2.capture_array()
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        self.faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)
        return self.faces
    
    def draw_faces(self):
        for (x, y, w, h) in self.faces:
            cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.imshow("Frame", self.frame)
        cv2.waitKey(1)

def servo_controller_auto(x, y, x_prev, y_prev):
    # Error calculation
    err_x = (frame_width / 2 - x) / frame_width
    err_y = (frame_height / 2 - y) / frame_height
    derr_x = (x_prev - x) / frame_width
    derr_y = (y_prev - y) / frame_height

    # P gain
    Kpx = 30
    Kpy = 30

    # D gain
    Kdx = 0
    Kdy = 0

    # PD control
    MVx = Kpx * err_x + Kdx * derr_x / dt
    MVy = Kpy * err_y + Kdy * derr_y / dt

    servo1.move(MVx)
    servo2.move(MVy)

# Constants
servo_pin_1 = 12  # GPIO 12
servo_pin_2 = 13  # GPIO 13

frame_width = 640
frame_height = 480
frame_rate = 30
dt = 0.1

auto_toggle = True

# main function
if __name__ == '__main__':
    # 1. Initialize & start servos
    servo1 = Servo(servo_pin_1)
    servo2 = Servo(servo_pin_2)
    time.sleep(2)
    servo1.move(90)
    servo2.move(45)

    # 2. Initialize camera
    camera = Camera(width=frame_width, height=frame_height, frame_rate=frame_rate)
    camera.start()
    time.sleep(1)

    # 3. Start irw subprocess
    process = subprocess.Popen(['irw'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    print("IR Remote Test Start...")

    # Main loop
    x_center_prev, y_center_prev = frame_width / 2, frame_height / 2
    def handle_irw():
        global auto_toggle
        while True:
            output = process.stdout.readline()
            if output:
                code = output.decode('utf-8').strip()
                print(f"Received code: {code}")
                if "KEY_0" in code:
                    auto_toggle = not auto_toggle
                if not auto_toggle:
                    if "KEY_2" in code:
                        servo2.move_continuous("increase")
                    elif "KEY_8" in code:
                        servo2.move_continuous("decrease")
                    elif "KEY_4" in code:
                        servo1.move_continuous("decrease")
                    elif "KEY_6" in code:
                        servo1.move_continuous("increase")
                    elif "KEY_5" in code:
                        servo1.stop()
                        servo2.stop()
            time.sleep(dt)
    
    irw_thread = threading.Thread(target=handle_irw)
    irw_thread.start()
    
    try:
        while True:
            # face detection and display
            faces = camera.get_face_coordinates()
            camera.draw_faces()
            max_area = 0
            x_center, y_center = float('nan'), float('nan')
            for face in faces:
                x, y, w, h = face
                if w * h > max_area:
                    max_area = w * h
                    x_center = x + w / 2
                    y_center = y + h / 2
                    
            camera.draw_faces()
            
            # Update x_center_prev and y_center_prev
            if not np.isnan(x_center) and not np.isnan(y_center):
                x_center_prev, y_center_prev = x_center, y_center
                
            # Auto mode
            if auto_toggle:
                if np.isnan(x_center) or np.isnan(y_center):
                    print("Invalid coordinates")
                else:
                    servo_controller_auto(x_center, y_center, x_center_prev, y_center_prev)
            
            time.sleep(dt)
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        process.terminate()
        camera.stop()
        cv2.destroyAllWindows()
        servo1.stop()
        servo2.stop()
        irw_thread.join()
        print("Resources released.")
