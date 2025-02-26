from time import sleep
from machine import I2C, Pin
from motors import Motor, Servo
from robot import Robot

"""
motorR = Motor(4, 5)
motorL = Motor(7, 6)

def onPress():
    print("pressed")
    startTime = time.time()
    print(startTime)
    timeDiff = 0
    
    while timeDiff < 2.65:
        timeDiff = time.time() - startTime
        
    print(time.time() - startTime)
    motorR.stop()
    motorL.stop()

while True:
    if button.value() == 1:
        onPress()
        break

"""



# I2c bus
i2c_bus = I2C(0, sda=Pin(16), scl=Pin(17))

pins = {'outer_sensorL_pin' : 18, 'outer_sensorR_pin' : 19,
        'line_sensorR_pin' : 20, 'line_sensorL_pin' : 21,
        'motorR_pwm_pin' : 4, 'motorR_dir_pin' : 5, # Motor Pins are Hard Coded in Robot()
        'motorL_pwm_pin' : 6, 'motorL_dir_pin' : 7, # Motor Pins are Hard Coded in Robot()
        'servo_pin1' : 13, 'servo_pin2' : 15,
        'led_pin' : 14, 'button_pin': 12 }

# Robot class
agv = Robot(i2c_bus, pins, phys_params={'turning_time': 1})

def onPress():
    print("Pressed")
    agv.forward(75, line_follow= True, junction_decision= False)

while True:
    if agv.button.value() == 1:
        onPress()
        break


'''
1. Observe colour of block
2. Initiate pick-up routine
2. Find shortest path from customer to delivery point in table
3. Initiate Backout routine
3. Use junction count and landmarks to track progress and when to turn
4. Once end of route is reached, initiate delivery routine
'''
   


    
