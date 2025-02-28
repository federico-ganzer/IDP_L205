from machine import Pin
from time import sleep
from sensors.tcs34725 import TCS34725
import utils
from pathfinder import dijkstra
from motors import Motor, Servo
from collections import deque


class Robot():
    
    def __init__(self, i2c_bus, pins, phys_params):
        
        
        # State Variables
        self.light = False # Boolean for light flashing
        self.current_node = 'START'
        self.current_target = None
        self.current_route = dijkstra(self.current_node, self.current_target) # List of nodes to visit
        self.current_direction = (0, 1) # starting direction as tuple
        self.current_destination = None
        self.block = False
        self.visited_customers = set()
        
        
        #physical parameters
        
        #time required to turn 90 degrees (with only one wheel active) = time required to turn 180 degrees with 2 wheels active
        self.turning_time = (3.14159*phys_params['axel_width'])/(2*phys_params['motor_max_speed']*phys_params['wheel_radius'])
        self.sensor_to_axel = phys_params['sensor_to_axel']
        
        #line following params
        
        self._window_size = 5
        self.left_sensor_hist = deque([0]*self._window_size, self._window_size)
        self.right_sensor_hist = deque([0]*self._window_size, self._window_size)
        self._prev_err = 0
        self._integral = 0
        self.kp = 20
        self.ki = 0.05
        self.kd = 1
        
        #I2C Sensors
        self.tcs = TCS34725(i2c_bus) # Colour Sensor
        
        #GPIO Connections
        self.button = Pin(pins['button_pin'], Pin.IN, Pin.PULL_DOWN)
        self.led = Pin(pins['led_pin'], Pin.OUT)
        self.line_sensorL = Pin(pins['line_sensorL_pin'], Pin.IN, Pin.PULL_DOWN) # Left Line Sensor
        self.line_sensorR = Pin(pins['line_sensorR_pin'], Pin.IN, Pin.PULL_DOWN) # Right Line Sensor
        self.outer_sensorL = Pin(pins['outer_sensorL_pin'], Pin.IN, Pin.PULL_DOWN) # Left Outer Sensor
        self.outer_sensorR = Pin(pins['outer_sensorR_pin'], Pin.IN, Pin.PULL_DOWN) # Right Outer Sensor
        
        #Motors and Servos
        self.motorR = Motor(4, 5) # Right Motor
        self.motorL = Motor(7, 6) # Left Motor
        self.servo1 = Servo(pins['servo_pin1']) # Servo 1
        self.servo2 = Servo(pins['servo_pin2']) # Servo 2

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
    
    
    def forward(self, speed, line_follow= True):
        '''
        Move the robot forward (CURRENTLY MOTOR TEST CODE)
        TODO: Implement line following algorithm
        '''
        while True:
            junction = self.detect_junction()
            if junction:
                if self.current_route is not None:
                    self.current_node = self.current_route.pop(0)
                #self.motorR.stop()
                #self.motorL.stop()
                decision = self.junction_decision()
                if decision != 0:
                    self.turn(junction, decision)
                
                '''
                 call decision
                 returns "left"[+], "right"[-] or "zero"
                 remove current node from route
                '''
                

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
        
        self.motorR.forward(100 - correction)
        self.motorL.forward(100 + correction)
        
        
    def turn(self, junction_type, decision):
        '''
        Turn the robot in the specified direction
        TODO: REVISE TURNING MOTOR CONTROL
        '''
        
        '''
        self.turning_time must be set based on motor speed and turning
        radius from phys_params - add max wheel speed to turn params
        
        '''
        
        
        if junction_type == 'L' or junction_type == 'T' and decision > 0: #sign must be the same for turn to be valid
            self.motorR.forward(80)
            self.motorL.stop()
            sleep(self.turning_time)
            
        elif junction_type == 'R' or junction_type == 'T' and decision < 0:
            self.motorR.stop()
            self.motorL.forward(80)
            sleep(self.turning_time) # define turning time based on turning radius
            
    
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
            self.current_route = dijkstra(self.current_node, target)
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
        
    def junction_decision(self):
        '''
        2D cross product between direction and the direction of next edge
        '''
        if self.current_route is not None:
            
            current_direction_x, current_direction_y = self.current_direction[0], self.current_direction[1]
            next_direction_x = self.current_route[2][0] - self.current_route[1][0]
            next_direction_y = self.current_route[2][1] - self.current_route[1][1]
            turn = current_direction_x * next_direction_y - current_direction_y * next_direction_x
            
            return turn
    
        
    
