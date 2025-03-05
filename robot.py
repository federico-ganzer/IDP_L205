from machine import Pin
from time import sleep
from sensors.tcs34725 import TCS34725
from sensors.vl53l0x import VL53L0X
import utils
from pathfinder import dijkstra
from motors import Motor, Servo
from collections import deque


class Robot():
    
    def __init__(self, i2c_bus_1, i2c_bus_2, pins, phys_params):
        
        
        # State Variables
        self.light = False # Boolean for light flashing
        
        self.current_node = 'START'
        self.current_target = 'A'
        
        route = dijkstra(self.current_node, self.current_target)
        self.current_route = route[0] if route is not None else None  # List of nodes to visit
        
        self.current_direction = (0, 1) # starting direction as tuple
        self.next_direction = None
        

        self.block = False
        self.visited_customers = set()
        
        
        #Physical parameters
        
        #time required to turn 90 degrees (with only one wheel active) = time required to turn 180 degrees with 2 wheels active
        self.turning_time = (3.14159*phys_params['axel_width'])/(2*phys_params['motor_max_speed']*phys_params['wheel_radius'])
        self.turning_prep_time = 0.5
        self.sensor_to_axel = phys_params['sensor_to_axel']
        
        self._speed = 100
        self._end_route = False
        
        #Line following params
        
        self._window_size = 5
        self.left_sensor_hist = deque([0]*self._window_size, self._window_size)
        self.right_sensor_hist = deque([0]*self._window_size, self._window_size)
        self.left_outer_sensor_hist = deque([0]*self._window_size, self._window_size)
        self.right_outer_sensor_hist = deque([0]*self._window_size, self._window_size)
        self._prev_err = 0
        self._integral = 0
        self.kp = 20
        self.ki = 0.05
        self.kd = 1
        
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

        if outer_left_avg > 0.8 and outer_right_avg > 0.8:
            return 'T'
        elif outer_left_avg > 0.8:
            return 'L'
        elif outer_right_avg > 0.8:
            return 'R'
        else:
            return False # No junction detected
    
    def forward(self, speed, line_follow= True):
        '''
        Move the robot forward (CURRENTLY MOTOR TEST CODE)
        
        '''
        self._speed = speed
        while not self._end_route:
            junction = self.detect_junction()
            if junction:
                if self.current_route is not None:
    
                    decision = self.junction_decision()
                
                    self.turn(junction, decision)
                    
                    self.current_node = self.current_route.pop(0)
                
                # BUG: don't know if i need to do this (if decision == 0 dont turn.)
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
        
        err = left_avg - right_avg
        diff = err - self._prev_err
        self._integral += err
        self._prev_err = err
        
        correction = self.kp*err + self.ki*self._integral + self.kd*diff
        
        self.motorR.forward(100 - correction)
        self.motorL.forward(100 + correction)
       
    def turn(self, junction_type, decision):
        
        '''
        Turn the robot in the specified direction defined by junction_type and the turn decision
        '''
        
        
        if junction_type == 'L' or junction_type == 'T' and decision > 0: #sign must be the same for turn to be valid
            # Moving average of the right sensor
            right_sensor_hist = deque([0]*10, 10)
            right_sensor_avg = self._get_moving_avg(right_sensor_hist)
            # moves forward slightly to allow room for turning
            self.motorR.forward(80)
            self.motorL.forward(80)
            sleep(self.turning_prep_time) # consider changing to a function of speed
            # starts turning without the check 
            self.motorR.forward(80)
            self.motorL.reverse(40)
            sleep(0.5)

            while right_sensor_avg < 0.8:
                self.motorR.forward(80)
                self.motorL.reverse(40)
                right_sensor_hist.append(self.line_sensorR.value())
                right_sensor_avg = self._get_moving_avg(right_sensor_hist)
            # update state once turn is complete
            if self.next_direction is not None:
                self.current_direction = self.next_direction
            self.forward(self._speed, line_follow= True)
            
        elif junction_type == 'R' or junction_type == 'T' and decision < 0:

            # Moving average of the left sensor
            left_sensor_hist = deque([0]*10, 10)
            left_sensor_avg = self._get_moving_avg(left_sensor_hist)
            # moves forward slightly to allow room for turning
            self.motorR.forward(80)
            self.motorL.forward(80)
            sleep(self.turning_prep_time)
            # starts turning without the check
            self.motorR.reverse(40)
            self.motorL.forward(80)
            sleep(0.5)

            while left_sensor_avg < 0.8:
                self.motorR.reverse(40)
                self.motorL.forward(80)
                left_sensor_hist.append(self.line_sensorL.value())
                left_sensor_avg = self._get_moving_avg(left_sensor_hist)
            # update state once turn is complete
            if self.next_direction is not None:
                self.current_direction = self.next_direction
            self.forward(self._speed, line_follow= True)

    def spin(self):
        '''
        Backout the robot until the sensors line up with the line again
        '''
        self.motorR.forward(100)
        self.motorL.reverse(100)
        sleep(self.turning_time)
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
            self.next_direction = (next_direction_x, next_direction_y)
            return turn
        
        if self.current_route is None or len(self.current_route) == 0:
            self._end_route = True
                     
    
    def pickup(self):
        if utils.check_centering(): # check might not be necessary
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
            
            self.motorR.forward(60)
            self.motorL.forward(60)
            sleep(0.2)

            #INFO: code for the distance sensor goes here... depending on the distance sensor data, we can go forwards, until the distance is a certain value. Once this has been done, then it will pick up the block using a servo.
            
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
                print(self.tof.ping() - 50, "mm")

                if self.tof.ping() - 50 < 10:
                    break

            
            #INFO: servo code... just twist 
            self.servo1.set_angle(15)

            cct, y = self.tcs.read()
            if cct is not None:
                target =  'DP1' if cct < 5000 else 'DP2'
            else:
                raise ValueError('Colour not detected')
            
            #update state
            self.current_target = target
            route = dijkstra(self.current_node, target)
            if route is not None:
                self.current_route = route[0] # list of nodes to visit
            self.block = True
            self.visited_customers.add(self.current_node)
            
            self.spin()

            return target
    
    def drop(self):
        '''
        Drop the block
        '''
        if self.current_node in set(['DP1', 'DP2']) and self.block:
            self.block = False
            self.spin()
            pass
            
