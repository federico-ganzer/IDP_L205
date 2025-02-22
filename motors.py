from machine import Pin, PWM
from utime import sleep

class Motor():
    def __init__(self, pwm_pin, dir_pin):
        self.pwm = PWM(Pin(pwm_pin)) # set speed
        self.dir = Pin(dir_pin, Pin.OUT) # set direction
        self.dir.value(0) # set direction to forward
        self.pwm.freq(1000) # set max frequency
        self.pwm.duty_u16(0) # set duty cycle

    def forward(self, speed):
        self.dir.value(0)
        self.pwm.duty_u16(speed)

    def reverse(self, speed):
        self.dir.value(1)
        self.pwm.duty_u16(speed)

    def stop(self):
        self.pwm.duty_u16(0)

class Servo():
    def __init__(self, pwm_pin):
        self.pwm = PWM(Pin(pwm_pin)) # set speed
        self.pwm.freq(50) # set max frequency
        self.pwm.duty_u16(0) # set duty cycle

    def set_angle(self, angle):
        self.pwm.duty_u16(int(65535 * angle / 180))

    def stop(self):
        self.pwm.duty_u16(0)
        
