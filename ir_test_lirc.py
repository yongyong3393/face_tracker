import subprocess
import time
import pigpio
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

    def stop(self):
        self.running = False
        self.pi.set_servo_pulsewidth(self.pin_number, 0)

def main():
    # Initialize servos
    servo1 = Servo(12)
    servo2 = Servo(13)
    time.sleep(2)

    # Start the servo
    servo1.move(90)
    servo2.move(30)
    time.sleep(1)

    print("IR Remote Test Start...")
    
    try:
        process = subprocess.Popen(['irw'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        while True:
            output = process.stdout.readline()
            if output:
                code = output.decode('utf-8').strip()
                print(f"Received code: {code}")

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
            time.sleep(0.1)
    except KeyboardInterrupt:
        process.terminate()
        print("Terminated")
    finally:
        servo1.stop()
        servo2.stop()

if __name__ == "__main__":
    main()
