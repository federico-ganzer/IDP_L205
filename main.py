from time import sleep
from machine import I2C, Pin
from sensors.tcs34725 import TCS34725
import utils
from pathfinder import find_route
from motors import Motor, Servo
from robot import Robot

# I2c bus
i2c_bus = I2C(0, sda=Pin(16), scl=Pin(17))

button_pin = 12
led_pin = 14
line_sensorL_pin = 6
line_sensorR_pin = 7
motorR_pwm_pin = 4
motorR_dir_pin = 5
motorL_pwm_pin = 6
motorL_dir_pin = 7

pins = [button_pin, led_pin, line_sensorL_pin,
        line_sensorR_pin, motorR_pwm_pin, motorR_dir_pin,
        motorL_pwm_pin, motorL_dir_pin]

# Robot class
agv = Robot(i2c_bus, pins)

'''
Place holder for testing:
TODO: Replace button press with line sensor
'''

led = Pin(14, Pin.OUT)
button = Pin(12, Pin.IN, Pin.PULL_DOWN)

while True:
  led.value(button.value())
  sleep(0.1)
  print(button.value())
    

'''
1. Observe colour of block
2. Initiate pick-up routine
2. Find shortest path from customer to delivery point in table
3. Initiate Backout routine
3. Use junction count and landmarks to track progress and when to turn
4. Once end of route is reached, initiate delivery routine
'''
   


    