
# Team L205 IDP

![License](https://img.shields.io/badge/license-MIT-blue.svg)  
![Version](https://img.shields.io/badge/version-1.0.0-brightgreen.svg)

## Table of Contents

1. [Introduction](#introduction)
2. [Modules](#modules)

## Introduction

## Modules

### Robot

**File:** `robot.py`

**Description:** Contains `Robot(i2c_bus, pins[dict], start, target1)` class. `Robot()` contains methods for AGV control for manouvring and collecting/depositing loads. Class is initialised with i2c pins and pin numbers associated with the servos, motors and sensors and can be easily modified as a modular system (eg. if sensor or servo additions are made). `start` and `target1` (in `node_id` form) are necessary to set the first route the `Robot()` should follow.

**Methods:**
- `forward()`: Drives robot motors at desired speed with optional line following and junction decision logic. Uses PID control for straight line following.

- `turn()`: Following a call to `junction_decision()`, the robot performs a 90 degree turn in the direction set by `junction_decision()`, provided that junction type also satisfies the turn. The current state variables are then updated. This checks that the middle sensor (on the side opposite to that of the direction of tha turn) is on the line and then exits the function, such that the PID control in `forward()` is reinitiated. Boolean `with_prep = True` determines if a turning prep-time should be added. During this prep-time the robot continues forward for a set time interval before turning. `with_prep` should be turned false if turning from a reverse position.

- `junction_decision()`: Using a 2D vector product between the current and next directions, the function returns a value that when positive indicates a left turn, when negative, a right turn, and when zero straight. If the `robot()` is at the end of the `current_route`, the robot stops.

- `detect_junction()`: Uses the two outer line sensors to detect junctions and the type of junction (`'R', 'L', 'T'`). This is then used to verify the validity of the turn in `turn(junction_type, decision)` and update the position of `current_node`(graphical representation of position) in `forward()`.

- `follow_line()`: Implements PID control with two line sensor inputs straddling the line to follow the line. PID weights can be adjusted in `self.kp`, `self.ki`, and `self.kd`.

- `spin()`: Performs a spin by 180 degrees. This will check that the configuration of the robot has now been realligned also (to an approximate degree) before the `follow_line()` takes over again.

- `pickup()`: Routine actuates the servo to pick up the block. The colour of the block is identified and the according depot is assigned as the `self.current_target`, creating a new route for `Robot()` to follow in forward. The `Robot()` will turn out of the customer zone accordingly.
- `drop()`: 

### Pathfinder
 
**File:** `pathfinder.py`

**Description:** Makes use of adjacency table of graph and Dijkstra's algorithm to find the minimal distance path from a given `start` node to `end` node. Includes a map from each node to their coordinate in physical space, which is used to determine which direction turns need to be made in.

**Functions:**
- `dijkstra(adj_list, start, end)`: Applies dijkstra's algorithm between `start` and `end` nodes (in `node_id` form) when prodived with `adj_list` of the underlying graph (map) and returns a tuple containing the list of the coordinates of nodes in a route (not including `start`) and the total length of the route.
- `convert_coord_to_node(coord)`: Uses the `coord_to_node` dictionary to convert phyical coordinates of nodes to their respective `node_id`.

### Sensors
**File:** `sensors.py`  
**Description:**  Includes classes and methods responsible for sensor reading

### Motors
**File:** `motors.py`
**Description:** Includes two class definitions that are used primarily in the `robot.py`, that controls the motor movements as well as servo movements. 

#### Motor(dir_pin, pwm_pin)
Takes parameters of the pins used, and has the following methods:
- `self.forward(speed)`
- `self.reverse(speed)`
- `self.stop()`


#### Servo(pwn_pin)
Takes parameters of the pin (singular) used. It has the following methods:
- `set_angle(angle)`
- `zero()`



## License

```
MIT License

Copyright (c) 2024 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---
