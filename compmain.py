from robot import Robot
from machine import I2C, Pin
from time import sleep

i2c_bus = I2C(0, sda= Pin(16), scl= Pin(17), freq= 400000)

pins = {'outer_sensorL_pin' : 18, 'outer_sensorR_pin' : 19,
        'line_sensorR_pin' : 20, 'line_sensorL_pin' : 21,
        'motorR_pwm_pin' : 4, 'motorR_dir_pin' : 5, # Motor Pins are Hard Coded in Robot()
        'motorL_pwm_pin' : 6, 'motorL_dir_pin' : 7, # Motor Pins are Hard Coded in Robot()
        'servo_pin1' : 13, 'servo_pin2' : 15,
        'led_pin' : 14, 'button_pin': 12 }

phys_params={'axel_width': 1, 'sensor_to_axel': 1, 'wheel_radius': 0.03,
             'motor_max_speed': 4.18879}

# Robot class
agv = Robot(i2c_bus, pins, phys_params) 
agv.current_target = 'A'
agv.current_node = 'START' # Test Route from START to A
agv.current_direction = (0, 1)
customers = set(['A', 'B', 'C', 'D'])


def onPress():
    
    agv.forward(75, line_follow= True)
    pass




if __name__ == "__main__":
    while True:
        if agv.button.value() == 1:
            onPress()
            break
    
