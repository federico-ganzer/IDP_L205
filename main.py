import time
from machine import I2C, Pin
from sensors.tcs34725 import TCS34725
import utils
from pathfinder import find_route

'''
1. Observe colour of block
2. Initiate pick-up routine
2. Find shortest path from customer to delivery point in table
3. Initiate Backout routine
3. Use junction count and landmarks to track progress and when to turn
4. Once end of route is reached, initiate delivery routine
'''

# Constants
sensor_distance = 0
wheel_distance = 0

# I2c bus
i2c_bus = I2C(0, sda=Pin(16), scl=Pin(17))

junction_coords = [(0, 1), (-1, 1), (-2, 1), (2, 1), (3, -2),
                   (3, 0), (3, 1), (3, 2), (4, 0), (5, 0)]


class Robot:
    def __init__(self):
        self.junction_count = 0
        self.current_route = []
        self.current_position = (0, 0)
        self.current_node = 'START'
        self.current_direction = 'N'
        self.current_target = None
        self.block = False
        self.visited = set()
        
        #sensors
        self.tcs = TCS34725(i2c_bus)

    def turn(self, new_direction):
        '''
        Turn the robot in the specified direction
        '''
        if utils.valid_turn(new_direction):
        
            if new_direction == 'N':
                pass
            elif new_direction == 'E':
                pass
            elif new_direction == 'S':
                pass
            elif new_direction == 'W':
                pass
    
    def backout(self):
        '''
        Backout the robot
        '''
        pass
    
    def pickup(self):
        if utils.check_centering():
            '''
            pick up the block
            '''
            cct, y = self.tcs.read('rgb')
            target =  'DP1' if cct < 5000 else 'DP2'
            
            #update state
            self.current_target = target
            self.current_route = find_route(self.current_position, target)
            self.block = True
            self.visited.add(self.current_position)
            return target
    
    def drop(self):
        '''
        Drop the block
        '''
        if self.current_position in set(['DP1', 'DP2']) and self.block:
            self.block = False
            self.backout()
            pass
            
        pass
    
    
    


    