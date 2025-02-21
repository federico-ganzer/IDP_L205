import heapq

'''
Each path defined in for of [start, end, path[list], length, nturns]
Twin of each path is also defined [end, start, -1*path[list][:-1], length, nturns]

'''
 # 'start_node_id': {connected_node_id: distance}
graph = { 'START': [('3', 1.0)],

            '1': [('DP2', 1.0), ('5', 2.0), ('2', 1.0)],

            '2': [('A', 1.0), ('3', 1.0), ('1', 1.0)],

            '3': [('START', 1.0), ('2', 1.0), ('4', 2.0)],

            'DP1': [('4', 1.0)],

            'DP2': [('1', 1.0)],

            '4': [('DP1', 1.0), ('3', 2.0), ('8', 2.0)],

            '5': [('1', 2.0), ('6', 2.0), ('10', 4.0)],

            '6': [('5', 2.0), ('7', 1.0), ('9', 1.0)],

            'A': [('2', 1.0)],

            'B': [('7', 1.0)],

            'C': [('9', 1.0)],

            'D': [('11', 1.0)],

            '7': [('6', 1.0), ('B', 1.0), ('8', 1.0)],

            '8': [('4', 2.0), ('7', 1.0), ('11', 3.0)],

            '9': [('6', 1.0), ('C', 1.0), ('10', 1.0)],

            '10': [('9', 1.0), ('5', 4.0), ('11', 1.0)],

            '11': [('10', 1.0), ('D', 1.0), ('8', 3.0)]
        }

def dijkstra(adj_list, start, end):
    '''
    Finds shortest path from start to end using Dijkstra's algorithm.
    '''
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
            return path[::-1]
        
        for neighbor, distance in adj_list.get(node, []):
            new_distance = current_distance + distance # modification can be made here to include turns
            if neighbor not in distances or new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                heapq.heappush(pq, (new_distance, neighbor))
                pred[neighbor] = node
                
    return None

#test
print(dijkstra(graph, 'C', 'DP1'))

# junction_id : (coordinate)


def find_route(start, end):
    
    return