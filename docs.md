
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
**Description:** Contains `Robot()` class. Robot contains methods for AGV control for manouvring and collecting/depositing loads. Class is initialised with pin numbers associated with the servos, motors and sensors and can be easily modified as a modular system (eg. if sensor or servo additions are made). 

### Pathfinder

**File:** `pathfinder.py`
**Description:** Makes use of adjacency table of graph and Dijkstra's algorithm to find the minimal distance path from a given `start` node to `end` node. Includes a map from each node to their coordinate in physical space, which is used to determine whic direction turns need to be made in.

**Functions:**
- `dijkstra(adj_list, start, end)`: Applies dijkstra's algorithm between `start` and `end` nodes when prodived with `adj_list` of the underlying graph (map)

### Sensors
**File:** `sensors.py`  
**Description:**  Includes classes and methods responsible for sensor reading

### other
**modules:**
- `utils.py`: Contains utility helper functions to check for validity of manouvres.


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
