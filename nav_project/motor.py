import RPi.GPIO as GPIO
import keyboard
import time
STOP  = 0
FORWARD  = 1
BACKWARD = 2

# 모터 채널
CH1 = 0
CH2 = 1

# PIN 입출력 설정
OUTPUT = 1
INPUT = 0

# PIN 설정
HIGH = 1
LOW = 0

# 실제 핀 정의
#PWM PIN
ENA = 26  #37 pin
ENB = 0   #27 pin

#GPIO PIN
IN1 = 19  #37 pin
IN2 = 13  #35 pin
IN3 = 6   #31 pin
IN4 = 5   #29 pin



# 핀 설정 함수
def setPinConfig(EN, INA, INB):        
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    # 100khz 로 PWM 동작 시킴 
    pwm = GPIO.PWM(EN, 100) 
    # 우선 PWM 멈춤.   
    pwm.start(0) 
    return pwm

# 모터 제어 함수
def setMotorContorl(pwm, INA, INB, speed, stat):

    #모터 속도 제어 PWM
    pwm.ChangeDutyCycle(speed)  
    
    if stat == BACKWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)
        
    #
    elif stat == FORWARD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)
        
    #정지
    elif stat == STOP:
        GPIO.output(INA, LOW)
        GPIO.output(INB, LOW)

def car_control(com):
    if com == 'go':
        setMotor(CH1, 80, FORWARD)
        setMotor(CH2, 80, FORWARD)
    elif com == 'back':
        setMotor(CH1, 80, BACKWARD)
        setMotor(CH2, 80, BACKWARD)   
    elif com == 'stop':
        setMotor(CH1, 100, STOP)
        setMotor(CH2, 100, STOP)  
    elif com == 'right':
        setMotor(CH1, 80, FORWARD)
        setMotor(CH2, 10, FORWARD)  
    elif com == 'left':
        setMotor(CH1, 10, FORWARD)
        setMotor(CH2, 80, FORWARD)  



def setMotor(ch, speed, stat):
    if ch == CH1:
        #핀 설정 후 pwm 핸들을 리턴 받음
        setMotorContorl(pwmA, IN1, IN2, speed, stat)
    else:
        #핀 설정 후 pwm 핸들을 리턴 받음
        setMotorContorl(pwmB, IN3, IN4, speed, stat)


if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM) 

    pwmA = setPinConfig(ENA, IN1, IN2)
    pwmB = setPinConfig(ENB, IN3, IN4)

    car_control('stop')

    while True:


    
    