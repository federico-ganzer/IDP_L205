from machine import Pin
from sensors.tcs34725 import TCS34725
import utils
from pathfinder import find_route
from motors import Motor, Servo

class Robot():
    
    def __init__(self, i2c_bus, pins):
        
        self.light = False # Boolean for light flashing
        self.current_route = []
        self.current_node = 'START'
        self.current_direction = 'N'
        self.current_destination = None
        self.block = False
        self.visited_customers = set()
        
        #I2C Sensors
        self.tcs = TCS34725(i2c_bus) # Colour Sensor
        
        #GPIO Connections
        self.led = Pin(pins[0], Pin.OUT)
        self.button = Pin(pins[1], Pin.IN, Pin.PULL_DOWN)
        self.line_sensorL = Pin(pins[6], Pin.IN, Pin.PULL_DOWN) # Left Line Sensor
        self.line_sensorR = Pin(pins[7], Pin.IN, Pin.PULL_DOWN) # Right Line Sensor
        #Motors 
        #TODO: Check if pins are correct before running
        self.motorR = Motor(pins[2], pins[3]) # Right Motor
        self.motorL = Motor(pins[4], pins[5]) # Left Motor

    def detect_junction(self):
        '''
        Detect junctions using outside line sensors
        '''
        pass
    
    def forward(self, speed):
        '''
        Move the robot forward (CURRENTLY MOTOR TEST CODE)
        TODO: Implement line following algorithm
        '''
        while not self.detect_junction():
            
            self.motorR.forward(speed)
            self.motorL.forward(speed)
            
        
    
    def turn(self, new_direction):
        '''
        Turn the robot in the specified direction
        '''
        if utils.valid_turn(new_direction): # remember to check if turn is valid
                return
    
    def backout(self):
        '''
        Backout the robot
        '''
        pass
    
    def pickup(self):
        if utils.check_centering():
            
            '''
            Pick up the block and read colour:
            1. Use ultrasonic sensor to check if block is in front of robot
            '''
            
            cct, y = self.tcs.read()
            if cct is not None:
                target =  'DP1' if cct < 5000 else 'DP2'
            else:
                raise ValueError('Colour not detected')
            
            #update state
            self.current_target = target
            self.current_route = find_route(self.current_node, target)
            self.block = True
            self.visited_customers.add(self.current_node)
            return target
    
    def drop(self):
        '''
        Drop the block
        '''
        if self.current_node in set(['DP1', 'DP2']) and self.block:
            self.block = False
            self.backout()
            pass
    
        
    
