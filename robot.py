from machine import Pin
from time import sleep
from sensors.tcs34725 import TCS34725
import utils
from pathfinder import find_route
from motors import Motor, Servo
from collections import deque


class Robot():
    
    def __init__(self, i2c_bus, pins, phys_params):
        
        
        # State Variables
        self.light = False # Boolean for light flashing
        self.current_route = []
        self.current_node = 'START'
        self.current_direction = 'N'
        self.current_destination = None
        self.block = False
        self.visited_customers = set()
        
        
        #physical parameters
        self.turning_time = phys_params['turning_time']
        self.axel_width = phys_params['axel_width']
        self.sensor_to_axel = phys_params['sensor_to_axel']
        
        #line following params
        
        self._window_size = 5
        self.left_sensor_hist = deque([0]*self._window_size, maxlen=self._window_size)
        self.right_sensor_hist = deque([0]*self._window_size, maxlen=self._window_size)
        self._prev_err = 0
        self._integral = 0
        self.kp = 1
        self.ki = 0
        self.kd = 0
        
        #I2C Sensors
        self.tcs = TCS34725(i2c_bus) # Colour Sensor
        
        #GPIO Connections
        self.button = Pin(pins[0], Pin.IN, Pin.PULL_DOWN)
        self.led = Pin(pins[1], Pin.OUT)
        self.line_sensorL = Pin(pins[2], Pin.IN, Pin.PULL_DOWN) # Left Line Sensor
        self.line_sensorR = Pin(pins[3], Pin.IN, Pin.PULL_DOWN) # Right Line Sensor
        self.outer_sensorL = Pin(pins[10], Pin.IN, Pin.PULL_DOWN) # Left Outer Sensor
        self.outer_sensorR = Pin(pins[11], Pin.IN, Pin.PULL_DOWN)
        
        
        #Motors 
        #TODO: Check if pins are correct before running
        self.motorR = Motor(pins[4], pins[5]) # Right Motor
        self.motorL = Motor(pins[6], pins[7]) # Left Motor
        self.servo1 = Servo(pins[8]) # Servo 1
        self.servo2 = Servo(pins[9]) # Servo 2

    def detect_junction(self):
        '''
        Detect junctions using outside line sensors
        '''
        if self.outer_sensorL.value() and self.outer_sensorR.value():
            return 'T'
        elif self.outer_sensorL.value():
            return 'L'
        elif self.outer_sensorR.value():
            return 'R'
        else:
            return False
    
    
    def forward(self, speed, line_follow=False):
        '''
        Move the robot forward (CURRENTLY MOTOR TEST CODE)
        TODO: Implement line following algorithm
        '''
        while True:
            Junction = self.detect_junction()
            if Junction:
                self.motorR.stop()
                self.motorL.stop()
                return Junction
            if line_follow:
                self.follow_line()
            else:
                self.motorR.forward(speed)
                self.motorL.forward(speed)
    
    
    def _get_moving_avg(self, sensor_hist):
        return sum(sensor_hist)/len(sensor_hist)
    
    def follow_line(self):
        '''
        Follow the line using line sensors
        1. Create moving average of prev 5 readings for each sensor
        2. Use PID to adjust motor speeds
        '''
        left_sensor = self.line_sensorL.value()
        right_sensor = self.line_sensorR.value()
        
        self.left_sensor_hist.append(left_sensor)
        self.right_sensor_hist.append(right_sensor)
        
        left_avg = self._get_moving_avg(self.left_sensor_hist)
        right_avg = self._get_moving_avg(self.right_sensor_hist)
        
        err = left_avg - right_avg
        diff = err - self._prev_err
        self._integral += err
        self._prev_err = err
        
        correction = self.kp*err + self.ki*self._integral + self.kd*diff
        
        self.motorR.forward(100 + correction)
        self.motorL.forward(100 - correction)
        
        
    def turn(self, new_direction):
        '''
        Turn the robot in the specified direction
        '''
        if utils.valid_turn(new_direction): # remember to check if turn is valid
            self.motorR.forward(100)
            self.motorL.reverse(100)
            sleep(self.turning_time/2)
            
    
    def spin(self):
        '''
        Backout the robot
        '''
        self.motorR.forward(100)
        self.motorL.reverse(100)
        sleep(self.turning_time)
        pass
    
    def pickup(self):
        if utils.check_centering(): # check might not be necessary
            
            '''
            Pick up the block and read colour:
            1. Use ultrasonic sensor to check if block is in front of robot
            2. Check alignment
            4. Turn on colour sensor
            5. forward (include in main as opposed to here?)
            6. activate lift
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
            self.spin()
            pass
    
        
    
