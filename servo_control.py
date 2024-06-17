import RPi.GPIO as GPIO # RPi.GPIO 라이브러리를 GPIO로 사용
from time import sleep  # time 라이브러리의 sleep함수 사용

servoPin          = 32   # 서보 핀
SERVO_MAX_DUTY    = 12   # 서보의 최대(180도) 위치의 duty
SERVO_MIN_DUTY    = 3    # 서보의 최소(0도) 위치의 duty

GPIO.setmode(GPIO.BOARD)        # GPIO 설정
GPIO.setup(servoPin, GPIO.OUT)  # 서보핀 출력으로 설정

servo = GPIO.PWM(servoPin, 50)  # 서보핀을 PWM 모드 50Hz로 사용하기 (50Hz > 20ms)
servo.start(0)  # 서보 PWM 시작 duty = 0, duty가 0이면 서보는 동작하지 않는다.


'''
서보 위치 제어 함수
degree에 각도를 입력하면 duty로 변환후 서보 제어(ChangeDutyCycle)
'''
def setServoPos(degree):
    # 각도는 180도를 넘을 수 없다.
    if degree > 180:
        degree = 180

    # 각도(degree)를 duty로 변경한다.
    duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
    # duty 값 출력
    print("Degree: {} to {}(Duty)".format(degree, duty))

    # 변경된 duty값을 서보 pwm에 적용
    servo.ChangeDutyCycle(duty)


if __name__ == "__main__":
    # set servo to 0 degree and sleep 1 second
    setServoPos(0)
    sleep(3)
    # increase servo degree 0 to 180
    for i in range(0, 181, 1):
        setServoPos(i)
        sleep(0.01)

    # 서보 PWM 정지
    servo.stop()
    # GPIO 모드 초기화
    GPIO.cleanup()