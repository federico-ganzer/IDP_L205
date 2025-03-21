from robot import Robot
from machine import I2C, Pin
from time import sleep
from pathfinder import dijkstra

i2c_bus_1 = I2C(0, sda= Pin(16), scl= Pin(17), freq= 400000)

pins = {'outer_sensorL_pin' : 18, 'outer_sensorR_pin' : 19,
        'line_sensorR_pin' : 20, 'line_sensorL_pin' : 21,
        'motorR_pwm_pin' : 4, 'motorR_dir_pin' : 5, # Motor Pins are Hard Coded in Robot()
        'motorL_pwm_pin' : 6, 'motorL_dir_pin' : 7, # Motor Pins are Hard Coded in Robot()
        'servo_pin1' : 13, 'servo_pin2' : 15,
        'led_pin' : 22, 'button_pin': 12 }

# Robot class
agv = Robot(i2c_bus_1, pins, start= 'BOX', target1= 'A') # Start at BOX and go to A
 # Test Route from START to A
customers = set(['A', 'B', 'C', 'D'])

def main():
    agv.servo1.zero() # set servo to zero 
    i=0
    while True:
        
        agv.forward(100)

        if agv.current_node in customers:
            agv.pickup() # backout() and turn() included.
            
        if agv.current_node in set(['DP1', 'DP2']):
            agv.drop() # spin() included
            
            min_distance = float('inf')
            
            for customer in customers - agv.visited_customers:# Cycle through not visited customers
                    result = dijkstra(agv.current_node, customer)
                    if result is not None:
                        path, distance = result
                        min_distance = min(min_distance, distance)
                        if min_distance == distance:
                            min_path = path
                            min_customer = customer
            
            agv.current_target = min_customer # current_target is set to closest customer to depot
            agv.current_route = min_path
            
        if agv.current_node in set(['DP1', 'DP2']) and agv.visited_customers == customers and not agv.block:
            
            if i > 0:
                agv.current_target = 'START'
                route = dijkstra(agv.current_node, 'START')
            else:
                agv.visited_customers = set()
                agv.current_target = 'A'
                route = dijkstra(agv.current_node, 'A')
                i = 1
            if route is not None:
                agv.current_route = route[0]
        
        
        if agv.current_node == 'START' and agv.current_target == 'START':
            agv.motorL.forward(50)
            agv.motorR.forward(50)
            sleep(2.4)
            agv.motorL.stop()
            agv.motorR.stop()
            break
            
            
if __name__ == "__main__":
    while True:
        if agv.button.value() == 1:
            main()
            break
    
'''
TODO: 
- Fix LED
- Reduce sleep time at customers
'''
