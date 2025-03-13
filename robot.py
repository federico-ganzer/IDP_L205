from machine import Pin
from time import sleep
from sensors.tcs34725 import TCS34725
from sensors.vl53l0x import VL53L0X
from pathfinder import dijkstra, convert_coord_to_node
from motors import Motor, Servo
from collections import deque


class Robot():
    
    def __init__(self, i2c_bus_1, pins, start, target1):
        
        
        # State Variables
        self.light = False # Boolean for light flashing
        self.current_node = start
        self.current_target = target1
        
        route = dijkstra(self.current_node, self.current_target)
        self.current_route = route[0] if route is not None else None  # List of nodes to visit
        
        self.current_direction = (0, 1) # starting direction as tuple
        self.next_direction = None
        

        self.block = False
        self.visited_customers = set()
        
        
        self.turning_prep_time = 0.5 # time required to move forward slightly before turning
        self._speed = 100
        
        #Line following params
        
        self._window_size = 5
        self.left_sensor_hist = deque([0]*self._window_size, self._window_size)
        self.right_sensor_hist = deque([0]*self._window_size, self._window_size)
        self.left_outer_sensor_hist = deque([0]*self._window_size, self._window_size)
        self.right_outer_sensor_hist = deque([0]*self._window_size, self._window_size)
        self._prev_err = 0
        self._integral = 0
        self.kp = 40
        self.ki = 0.03
        self.kd = 3
        
        #I2C Sensors
        self.tcs = TCS34725(i2c_bus_1) # Colour Sensor
        
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
        Detect junctions using outside line sensors\\
        Returns "junc" if a junction is detected, otherwise returns False\\
        '''

        outer_left_sensor = self.outer_sensorL.value()
        outer_right_sensor = self.outer_sensorR.value()
        
        self.left_outer_sensor_hist.append(outer_left_sensor)
        self.right_outer_sensor_hist.append(outer_right_sensor)

        outer_left_avg = self._get_moving_avg(self.left_outer_sensor_hist)
        outer_right_avg = self._get_moving_avg(self.right_outer_sensor_hist)

        if outer_left_avg > 0.8 or outer_right_avg > 0.8:
            return "junc"
        else:
            return False # No junction detected
    
    def forward(self, speed):
        '''
        Move the robot forward\\
        Continuously detects junctions and follows the line\\
        Stops the robot when the route is completed or an end condition is met\\
        No return value\\
        '''
        self._speed = speed
        
        while True:
            
            if self.visited_customers == set() and not self.block and self.current_node == '3':
                self.led.value(1)
            
            if self.visited_customers == set(['A', 'B', 'C', 'D']) and self.current_node == '3' and not self.block and self.current_target == 'START':
                self.led.value(0)
            
            
            junction = self.detect_junction()
            if junction == "junc":
                if self.current_route is not None:
    
                    '''
                     call decision
                     returns "left"[+], "right"[-] or "zero"
                     remove current node from route
                    '''
                    
                    decision = self.junction_decision()
                    
                    if decision is False: # end of route
                        self.current_node = convert_coord_to_node(self.current_route.pop(0)) # update current node
                        break
                    
                    self.turn(decision)
                    junction = False
                    
                    self.current_node = convert_coord_to_node(self.current_route.pop(0)) # update current node
                    
            self.follow_line()
        
        self.motorL.stop()
        self.motorR.stop()
       
    def _get_moving_avg(self, sensor_hist):
        '''
        Calculate the moving average of sensor readings\\
        Returns the moving average of the sensor readings\\
        '''
        return sum(sensor_hist)/len(sensor_hist)
    
    def follow_line(self):
        '''
        Follow the line using line sensors\\
        Uses PID control to adjust motor speeds based on sensor readings\\
        No return value\\
        '''
        left_sensor = self.line_sensorL.value()
        right_sensor = self.line_sensorR.value()
        
        self.left_sensor_hist.append(left_sensor)
        self.right_sensor_hist.append(right_sensor)
        
        left_avg = self._get_moving_avg(self.left_sensor_hist)
        right_avg = self._get_moving_avg(self.right_sensor_hist)
        
        err = right_avg - left_avg
        diff = err - self._prev_err
        self._integral += err
        self._prev_err = err
        
        correction = self.kp*err + self.ki*self._integral + self.kd*diff
        
        self.motorR.forward(100 - correction)
        self.motorL.forward(100 + correction)
       
    def turn(self, decision, with_prep = True):
        '''
        Turn the robot in the specified direction defined by the turn decision \\
        Parameters: \\
            decision: positive for left turn, negative for right turn, zero for straight \\
            with_prep: boolean indicating whether to move forward slightly before turning \\
        Updates the current direction after the turn \\
        No return value
        '''
        
        if decision > 0: # left turn
            
            # Moving average of the right sensor
            right_sensor_hist = deque([0]*10, 10)
            right_sensor_avg = self._get_moving_avg(right_sensor_hist)
            # moves forward slightly to allow room for turning
            if with_prep:
                self.motorR.forward(80)
                self.motorL.forward(80)
                sleep(self.turning_prep_time) # consider changing to a function of speed
            # starts turning without the check 
            self.motorR.forward(80)
            self.motorL.reverse(80)
            sleep(0.5)

            while right_sensor_avg < 0.8:
                self.motorR.forward(80)
                self.motorL.reverse(80)
                right_sensor_hist.append(self.line_sensorR.value())
                right_sensor_avg = self._get_moving_avg(right_sensor_hist)
            # update state once turn is complete
            if self.next_direction is not None:
                self.current_direction = self.next_direction
        
        elif decision < 0: # right turn
            
            # Moving average of the left sensor
            left_sensor_hist = deque([0]*10, 10)
            left_sensor_avg = self._get_moving_avg(left_sensor_hist)
            # moves forward slightly to allow room for turning
            if with_prep:
                self.motorR.forward(80)
                self.motorL.forward(80)
                sleep(self.turning_prep_time)
            # starts turning without the check
            self.motorR.reverse(80)
            self.motorL.forward(80)
            sleep(0.5)

            while left_sensor_avg < 0.8:
                self.motorR.reverse(80)
                self.motorL.forward(80)
                left_sensor_hist.append(self.line_sensorL.value())
                left_sensor_avg = self._get_moving_avg(left_sensor_hist)
            # update state once turn is complete
            if self.next_direction is not None:
                self.current_direction = self.next_direction
        elif decision == 0: # straight
            sleep(self.turning_prep_time) # just so it doesn't detect multiple straight junctions in one whilst going straight
            
    def spin(self, speed, direction):
        '''
        Spin the robot in place until the sensors line up with the line again\\
        Parameters:\\
            speed: speed of the motors\\
            direction: positive for left spin, negative for right spin\\
        No return value
        '''
        # BUG: Need to check if this is the right way around
        
        
        if direction > 0: # left spin
            
            # Moving average of the right sensor
            right_sensor_hist = deque([0]*10, 10)
            right_sensor_avg = self._get_moving_avg(right_sensor_hist)
            # moves forward slightly to allow room for turning
            # consider changing to a function of speed
            # starts turning without the check 
            self.motorR.forward(speed)
            self.motorL.reverse(speed)
            sleep(0.5)

            while right_sensor_avg < 0.8:
                self.motorR.forward(speed)
                self.motorL.reverse(speed)
                right_sensor_hist.append(self.line_sensorR.value())
                right_sensor_avg = self._get_moving_avg(right_sensor_hist)
        
        if direction < 0: # right spin
            
            # Moving average of the right sensor
            left_sensor_hist = deque([0]*10, 10)
            left_sensor_avg = self._get_moving_avg(left_sensor_hist)
            # moves forward slightly to allow room for turning
            # consider changing to a function of speed
            # starts turning without the check 
            self.motorR.forward(speed)
            self.motorL.reverse(speed)
            sleep(0.5)

            while left_sensor_avg < 0.8:
                self.motorR.forward(speed)
                self.motorL.reverse(speed)
                left_sensor_hist.append(self.line_sensorL.value())
                left_sensor_avg = self._get_moving_avg(left_sensor_hist)
        
        self.current_direction = (0, 1) 
                               
        pass

    def back_out(self, speed, node):
        '''
        Back out the robot from the current position\\
        Parameters:\\
            speed: speed of the motors\\
            node: current node to determine the time for reversing\\
        No return value
        '''
        time_for_reverse = {
            "A": 1.3,
            "B": 0.9,
            "C": 1.3,
            "D": 0.9,
            "DP1": 2,
            "DP2": 2
        }

        self.motorL.reverse(speed)
        self.motorR.reverse(speed)
        sleep(time_for_reverse[node])
    
    def junction_decision(self):
        '''
        Determine the direction of the turn at a junction \\
        Uses 2D cross product between the current direction and the direction of the next edge\\
        Returns the turn decision: positive for left, negative for right, zero for straight, False if no next direction \\
        '''

        if self.current_route is not None and len(self.current_route) > 1: # (used to be 2)
            # if len(self.current_route) == 1, then the next direction does not exist because the next node is the target.
            current_direction_x, current_direction_y = self.current_direction[0], self.current_direction[1]
            
            next_direction_x = self.current_route[1][0] - self.current_route[0][0]
            next_direction_y = self.current_route[1][1] - self.current_route[0][1]
        
            turn = current_direction_x * next_direction_y - current_direction_y * next_direction_x
            self.next_direction = (next_direction_x, next_direction_y)
            return turn
        else: # next direction does not exist
            return False
                               
    def pickup(self):
        '''
        Pick up a block and determine its color \\
        Updates the current target based on the color detected \\
        Updates the route and state variables after picking up the block \\
        No return value
        '''

        # servo twisting 
        self.servo1.set_angle(15)

        cct_hist = deque([0]*50, 50)
        temp_i = 0
        while temp_i < 100:
            cct, y = self.tcs.read()
            if cct is not None:
                cct_hist.append(int(cct))
            else: # just so it goes to a depot.. doesn't matter which one
                self.current_target = 'DP1' 
                raise ValueError('Colour not detected')
            temp_i += 1

        cct_avg = self._get_moving_avg(cct_hist)
        self.current_target = 'DP1' if cct_avg < 6000 else 'DP2'
        
        # update state
        route = dijkstra(self.current_node, self.current_target)
        if route is not None:
            
            self.current_route = route[0] # list of nodes to visit
            self.block = True
            self.visited_customers.add(self.current_node)

            self.back_out(80, self.current_node)

            direction = self.junction_decision()

            #self.spin(80, direction)
            self.turn(direction, with_prep= False)

            self.current_node = convert_coord_to_node(self.current_route.pop(0))
        
        return

    def drop(self):
        '''
        Drop the block at the current depot\\
        Updates the state variables after dropping the block\\
        No return value\\
        '''
        if self.current_node in set(['DP1', 'DP2']) and self.block:
            self.servo1.set_angle(0)
            self.block = False
            
        self.motorR.forward(50)
        self.motorL.forward(50)

        self.servo1.set_angle(0)
    

        self.motorR.reverse(80)
        self.motorL.reverse(80)
        sleep(0.5)
                
        '''
        while not (outer_left_avg > 0.8 and outer_right_avg > 0.8):
            if outer_left_avg > outer_right_avg:
                self.motorR.reverse(80)
                self.motorL.stop()
            elif outer_left_avg < outer_right_avg: 
                self.motorL.reverse(80)
                self.motorR.stop()
            elif outer_left_avg == outer_right_avg:
                self.motorR.reverse(80)
                self.motorL.reverse(80)
        '''
        
        self.block = False
        
        direction = 1 if self.current_node == 'DP1' else -1 # set spin direction such that it turns towards inside
            
        self.spin(80, direction)
        # update route after the block has been dropped
        

'''
TODO:
- Test compmain.py
'''
