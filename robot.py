from machine import Pin
from time import sleep
from sensors.tcs34725 import TCS34725
from sensors.vl53l0x import VL53L0X
from pathfinder import dijkstra, convert_coord_to_node
from motors import Motor, Servo
from collections import deque


class Robot():
    
    def __init__(self, i2c_bus_1, i2c_bus_2, pins, phys_params, start, target1):
        
        
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
        
        
        #Physical parameters
        
        #time required to turn 90 degrees (with only one wheel active) = time required to turn 180 degrees with 2 wheels active
        self.turning_time = (3.14159*phys_params['axel_width'])/(2*phys_params['motor_max_speed']*phys_params['wheel_radius'])
        self.turning_prep_time = 0.65 # time required to move forward slightly before turning
        self.sensor_to_axel = phys_params['sensor_to_axel']
        
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
        self.tof = VL53L0X(i2c_bus_2) # ToF sensor
        
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
        '''
        while loop --> moving avg. of (large amount) readings --> threshold --> if statements
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
        Move the robot forward (CURRENTLY MOTOR TEST CODE)
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
        
        err = right_avg - left_avg
        diff = err - self._prev_err
        self._integral += err
        self._prev_err = err
        
        correction = self.kp*err + self.ki*self._integral + self.kd*diff
        
        self.motorR.forward(100 - correction)
        self.motorL.forward(100 + correction)
       
    def turn(self, decision, with_prep = True): # NOTE: to self, junction_type is not accessed 
        '''
        Turn the robot in the specified direction defined by junction_type and the turn decision
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
        Backout the robot until the sensors line up with the line again
        '''
        # BUG: Need to check if this is the right way around


        if direction < 0:
            self.motorR.reverse(speed)
            self.motorL.forward(speed)
        sleep(self.turning_time)
        
        
        if direction > 0: # left turn
            
            # Moving average of the right sensor
            right_sensor_hist = deque([0]*10, 10)
            right_sensor_avg = self._get_moving_avg(right_sensor_hist)
            # moves forward slightly to allow room for turning
            # consider changing to a function of speed
            # starts turning without the check 
            self.motorR.forward(80)
            self.motorL.reverse(80)
            sleep(0.5)

            while right_sensor_avg < 0.8:
                self.motorR.forward(80)
                self.motorL.reverse(80)
                right_sensor_hist.append(self.line_sensorR.value())
                right_sensor_avg = self._get_moving_avg(right_sensor_hist)
        
        if direction < 0: # left turn
            
            # Moving average of the right sensor
            left_sensor_hist = deque([0]*10, 10)
            left_sensor_avg = self._get_moving_avg(left_sensor_hist)
            # moves forward slightly to allow room for turning
            # consider changing to a function of speed
            # starts turning without the check 
            self.motorR.forward(80)
            self.motorL.reverse(80)
            sleep(0.5)

            while left_sensor_avg < 0.8:
                self.motorR.forward(80)
                self.motorL.reverse(80)
                left_sensor_hist.append(self.line_sensorL.value())
                left_sensor_avg = self._get_moving_avg(left_sensor_hist)
        
        self.current_direction = (0, 1) 
                               
        pass

    def back_out(self, speed, node):
        time_for_reverse = {
            "A": 2,
            "B": 2,
            "C": 2,
            "D": 1,
            "DP1": 2,
            "DP2": 2
        }

        self.motorL.reverse(speed)
        self.motorR.reverse(speed)
        sleep(time_for_reverse[node])
    
    def junction_decision(self):
        '''
        2D cross product between direction and the direction of next edge
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
                               
    def pickup(self, current_pickup_point):
        '''
        Go forwards until the robot is right in front of the block
        
        Pick up the block and read colour:
        1. Use ultrasonic sensor to check if block is in front of robot
        2. Check alignment
        4. Turn on colour sensor
        5. forward (include in main as opposed to here?)
        6. activate lift
        Now need to spin
        then go past the lines
        then go to back to forward method
        '''
        '''
        # Actual pick up of the block
        self.motorR.forward(60)
        self.motorL.forward(60)
        sleep(0.2)
        # INFO: code for the distance sensor goes here... depending on the distance sensor data, we can go forwards, until the distance is a certain value. Once this has been done, then it will pick up the block using a servo.
        # We might not need to use this ToF sensor, if the line sensing is good enough
        # following code was copied from the default given to us
        budget = self.tof.measurement_timing_budget_us
        print("Budget was:", budget)
        self.tof.set_measurement_timing_budget(400000)
        # Sets the VCSEL (vertical cavity surface emitting laser) pulse period for the 
        # given period type (VL53L0X::VcselPeriodPreRange or VL53L0X::VcselPeriodFinalRange) 
        # to the given value (in PCLKs). Longer periods increase the potential range of the sensor. 
        # Valid values are (even numbers only):
        # tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0], 18)
        self.tof.set_Vcsel_pulse_period(self.tof.vcsel_period_type[0], 12)
        # tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1], 14)
        self.tof.set_Vcsel_pulse_period(self.tof.vcsel_period_type[1], 8)
        while True:
        # Start ranging
            ping = self.tof.ping()
            
            if ping is not None:
                print(ping - 50, "mm")
                if ping - 50 < 10:
                    self.motorR.stop()
                    self.motorL.stop()
                    break
        '''
        
        # servo twisting 
        self.servo1.set_angle(15)

        cct, y = self.tcs.read()
        if cct is not None:
            self.current_target =  'DP1' if cct < 5000 else 'DP2'
        else: # just so it goes to a depot.. doesn't matter which one
            self.current_target = 'DP1' 
            raise ValueError('Colour not detected')
        
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
        Drop the block
        '''
        if self.current_node in set(['DP1', 'DP2']) and self.block:
            self.servo1.set_angle(0)
            self.block = False
            
        direction = 1 if self.current_node == 'DP1' else -1 # set spin direction such that it turns towards inside
            
        self.spin(80, direction)
        # update route after the block has been dropped
        



'''
TODO:
- Junction detection: updating of current_nodes during / after a turn
'''
