import heapq

'''
Each path defined in for of [start, end, path[list], length, nturns]
Twin of each path is also defined [end, start, -1*path[list][:-1], length, nturns]

TODO: ADD CORNER NODES
'''


 # 'start_node_id': {connected_node_id: distance}
adj_list = { 'START': [('3', 1.0)],

            '1': [('DP1', 1.0), ('5', 2.0), ('2', 1.0)],

            '2': [('A', 1.0), ('3', 1.0), ('1', 1.0)],

            '3': [('START', 1.0), ('2', 1.0), ('4', 2.0)],

            'DP2': [('4', 1.0)],

            'DP1': [('1', 1.0)],

            '4': [('DP2', 1.0), ('3', 2.0), ('8', 2.0)],

            '5': [('1', 2.0), ('6', 2.0), ('10', 2.0)],

            '6': [('5', 2.0), ('7', 1.0), ('9', 1.0)],

            'A': [('2', 1.0)],

            'B': [('7', 1.0)],

            'C': [('9', 1.0)],

            'D': [('12', 1.0)],

            '7': [('6', 1.0), ('B', 1.0), ('8', 1.0)],

            '8': [('4', 2.0), ('7', 1.0), ('12', 3.0)],

            '9': [('6', 1.0), ('C', 1.0), ('11', 1.0)],
            
            '10': [('5', 2.0), ('11', 2.0)],

            '11': [('9', 1.0), ('5', 4.0), ('12', 1.0)],

            '12': [('11', 1.0), ('D', 1.0), ('13', 1.0)],
            
            '13': [('12', 1.0) , ('8', 2.0)]
            }

node_to_coord = { 'START': (0, -1),
                  '1': (-2, 0),
                  '2': (-1, 0),
                  '3': (0, 0),
                  'DP1': (-2, -1),
                  'DP2': (2, -1),
                  '4': (2, 0),
                  '5': (-2, 2),
                  '6': (0, 2),
                  'A': (-1, 1),
                  'B': (1, 1),
                  'C': (-1, 3),
                  'D': (1, 3),
                  '7': (1, 2),
                  '8': (2, 2),
                  '9': (0, 3),
                  '10': (-2, 4),
                  '11': (0, 4),
                  '12': (1, 4),
                  '13' : (2, 4) 
                }

def dijkstra(start, end):
    '''
    Finds shortest path from start to end using Dijkstra's algorithm.
    '''
    if start not in adj_list or end not in adj_list:
        return None
    
    pq = [(0, start)]
    distances = {start: 0}
    pred = {start: None}
    
    while pq:
        current_distance, node = heapq.heappop(pq)
        
        if node == end:
            path = []
            while node is not None:
                path.append(node)
                node = pred[node]
            return [node_to_coord[node] for node in path[::-1]]
        
        for neighbor, distance in adj_list.get(node, []):
            new_distance = current_distance + distance # modification can be made here to include turns
            if neighbor not in distances or new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                heapq.heappush(pq, (new_distance, neighbor))
                pred[neighbor] = node
                
    return None

#test
#print(dijkstra('D', 'DP2'))

