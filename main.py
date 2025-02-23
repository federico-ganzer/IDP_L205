from time import sleep
from machine import I2C, Pin
from sensors.tcs34725 import TCS34725
import utils
from pathfinder import find_route
from motors import Motor, Servo
from robot import Robot

# I2c bus
#i2c_bus = I2C(0, sda=Pin(16), scl=Pin(17))
#pins = []
# Robot class
# agv = Robot(i2c_bus, pins)

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
   


    