import pigpio
from picamera2 import Picamera2
import cv2
from time import sleep
import numpy as np

class Servo:
    # Pulse width between 500us to 2500us
    min_pulsewidth = 650
    max_pulsewidth = 2650

    def __init__(self, pin_number):
        self.pin_number = pin_number
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise IOError("Failed to connect to pigpio daemon")

    def move(self, angle):
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180
        print('Moving servo to', angle)
        pulsewidth = Servo.min_pulsewidth + (angle / 180.0) * (Servo.max_pulsewidth - Servo.min_pulsewidth)
        self.pi.set_servo_pulsewidth(self.pin_number, pulsewidth)

    def stop(self):
        self.pi.set_servo_pulsewidth(self.pin_number, 0)  # Stop servo pulses

# Create servo1, servo2 objects
servo1 = Servo(12)
servo2 = Servo(13)

# Move servos
servo1.move(60)
servo2.move(0)
sleep(3)

# move servos
servo1.move(90)
servo2.move(90)
sleep(2)
servo1.move(60)
servo2.move(60)
sleep(2)
servo1.move(45)
servo2.move(45)
sleep(2)
servo1.move(0)
servo2.move(0)
sleep(1)

servo1.stop()
servo2.stop()