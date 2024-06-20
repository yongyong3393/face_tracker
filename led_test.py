import pigpio
import time

pi = pigpio.pi()
if not pi.connected:
    raise IOError("Failed to connect to pigpio daemon")



LED_PIN = 25
pi.set_mode(LED_PIN, pigpio.OUTPUT)

try:
    while True:
        pi.write(LED_PIN, 1)
        time.sleep(1)
        pi.write(LED_PIN, 0)
        time.sleep(1)
except KeyboardInterrupt:
    print("Test End")
    
finally:
    pi.write(LED_PIN, 0)
    pi.stop
    print("Resource released")
