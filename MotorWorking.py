from machine import Pin , PWM
from utime import sleep

dir1 = Pin(4,Pin.OUT)
dir2 = Pin(7,Pin.OUT)
pwm1 = PWM(Pin(5))
pwm2 = PWM(Pin(6))
pwm1.freq(1000)
pwm2.freq(1000)

def RotateCW(duty):
    dir1.value(1)
    duty_16 = int((duty*65536)/100)
    pwm1.duty_u16(duty_16)
    dir2.value(1)
    duty_16 = int((duty*65536)/100)
    pwm2.duty_u16(duty_16)

def RotateCCW(duty):
    dir1.value(0)
    duty_16 = int((duty*65536)/100)
    pwm1.duty_u16(duty_16)
    dir2.value(0)
    duty_16 = int((duty*65536)/100)
    pwm2.duty_u16(duty_16)
    
def StopMotor():
    dir1.value(0)
    pwm1.duty_u16(0)
    dir2.value(0)
    pwm2.duty_u16(0)    

while True:
    RotateCW(100)
    sleep(5)
    RotateCCW(100)
    sleep(5)
    StopMotor()

  

