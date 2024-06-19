import pigpio
import time

# GPIO 핀 번호 설정
IR_PIN = 17
DEBOUNCE_TIME = 0.2  # 200ms

last_time = 0

def ir_callback(gpio, level, tick):
    global last_time
    current_time = time.time()
    if current_time - last_time > DEBOUNCE_TIME:
        print("IR signal received")
        last_time = current_time

# pigpio 초기화
pi = pigpio.pi()
if not pi.connected:
    exit()

pi.set_mode(IR_PIN, pigpio.INPUT)
pi.callback(IR_PIN, pigpio.EITHER_EDGE, ir_callback)

try:
    print("IR receiver test started. Press Ctrl+C to exit.")
    while True:
        time.sleep(1)  # 메인 루프에서 1초 대기

except KeyboardInterrupt:
    print("Exiting program")

finally:
    pi.stop()  # pigpio 정리
