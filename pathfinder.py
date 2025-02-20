'''
Each path defined in for of [start, end, path[list], length, nturns]
Twin of each path is also defined [end, start, -1*path[list][:-1], length, nturns]

'''

class Junction(): #Node
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.connections = []
        
    def add_connection(self, junction):
        self.connections.append(junction)
        
    def __str__(self):
        return self.id
        
    def __repr__(self):
        return self.id

class Map():
    def __init__(self):
        self.junctions = set()
    
    def add_junction(self, junction):
        self.junctions.add(junction)
    
    def add_road(self, junction1, junction2):
        self.add_junction(junction1)
        self.add_junction(junction2)
        junction1.add_connection(junction2)
        junction2.add_connection(junction1)


def find_route(start, end):
    '''
    Find the shortest path from start to end using BFS
    '''
    return