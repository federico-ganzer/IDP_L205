from robot import Robot
from machine import I2C, Pin
from time import sleep
from pathfinder import dijkstra

i2c_bus_1 = I2C(0, sda= Pin(16), scl= Pin(17), freq= 400000)
i2c_bus_2 = I2C(1, sda= Pin(16), scl= Pin(17), freq= 400000)

pins = {'outer_sensorL_pin' : 18, 'outer_sensorR_pin' : 19,
        'line_sensorR_pin' : 20, 'line_sensorL_pin' : 21,
        'motorR_pwm_pin' : 4, 'motorR_dir_pin' : 5, # Motor Pins are Hard Coded in Robot()
        'motorL_pwm_pin' : 6, 'motorL_dir_pin' : 7, # Motor Pins are Hard Coded in Robot()
        'servo_pin1' : 13, 'servo_pin2' : 15,
        'led_pin' : 14, 'button_pin': 12 }

phys_params={'axel_width': 1, 'sensor_to_axel': 1, 'wheel_radius': 0.03,
             'motor_max_speed': 4.18879}

# Robot class
agv = Robot(i2c_bus_1, i2c_bus_2, pins, phys_params, start= 'BOX', target1= 'A') # Start at BOX and go to A
 # Test Route from START to A
customers = set(['A', 'B', 'C', 'D'])

def main():
    
    while True:
        agv.forward(75) # TODO: need to put in an exit path within the forward program for this to function properly
        
        if agv.visited_customers == set() and not agv.block and agv.current_node == '3':
            agv.led.value(1) # Turn on LED when AGV first starts at node 3 this can be changed to when it leaves the box
            
        if agv.current_node in customers:
            agv.pickup() # spin() included
        
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
            agv.current_target = 'START'
            route = dijkstra(agv.current_node, 'START')
            if route is not None:
                agv.current_route = route[0]
        
        if agv.visited_customers == customers and agv.current_node == '3' and not agv.block and agv.current_target == 'START':
            agv.led.value(0) # Turn off LED when AGV reaches node 3 and is ready to go back to START
        
        if agv.current_node == 'START' and agv.current_target == 'START':
            agv.forward(50) # might need to just use motor control directly
            sleep(2)
            agv.motorL.stop()
            agv.motorR.stop()
            break
            
            
            
        
                               

if __name__ == "__main__":
    while True:
        if agv.button.value() == 1:
            main()
            break
    
