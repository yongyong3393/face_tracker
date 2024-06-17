import RPi.GPIO as GPIO
from time import sleep

def initialize_board():
    # set pin numbering system
    GPIO.setmode(GPIO.BOARD)

    # set servo pins
    servo_pin_1 = 32
    servo_pin_2 = 33
    GPIO.setup(servo_pin_1, GPIO.OUT)
    GPIO.setup(servo_pin_2, GPIO.OUT)

    # make 50 Hz pwm
    servo1_pwm = GPIO.PWM(servo_pin_1, 50)
    servo2_pwm = GPIO.PWM(servo_pin_2, 50)

    return servo1_pwm, servo2_pwm

class Servo:
    min_duty = 3
    max_duty = 12

    def __init__(self, servo_pwm):
        self.pwm = servo_pwm

    def start(self):
        self.pwm.start(0)

    def move(self, theta):
        print(theta)
        if theta < 0:
            theta = 0
        elif theta > 180:
            theta = 180        
        duty = Servo.min_duty + (Servo.max_duty - Servo.min_duty) * theta / 180.0
        self.pwm.ChangeDutyCycle(duty)
    
    def stop(self):
        self.pwm.stop()

def move_servo(theta, phi):
    pass

if __name__ == "__main__":

    servo1_pwm, servo2_pwm = initialize_board()
    
    # Create servo1, servo2 objects
    servo1 = Servo(servo1_pwm)
    servo2 = Servo(servo2_pwm)

    # Start servo
    servo1.start()
    servo2.start()

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

    # Stop servos
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()