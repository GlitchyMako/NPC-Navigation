from heapq import heappush, heappop
from math import sqrt

def manhattan_distance(position1, position2):
    return abs(position1[0] - position2[0]) + abs(position1[1] - position2[1]) + abs(position1[2] - position2[2])

def euclidean_distance(position1, position2):
    return sqrt((position1[0] - position2[0])**2 + (position1[1] - position2[1])**2 + (position1[2] - position2[2])**2)

def astar(graph, start, end):
    heap = [(0, start)]
    visited = set()
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while heap:
        current_cost, current_node = heappop(heap)
        
        if current_node == end:
            break
        
        if current_node in visited:
            continue
        
        visited.add(current_node)
        
        for neighbor in graph.neighbors(current_node):
            new_cost = cost_so_far[current_node] + graph.cost(current_node, neighbor)
            
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + euclidean_distance(end, neighbor)
                heappush(heap, (priority, neighbor))
                came_from[neighbor] = current_node
    
    return came_from, cost_so_far

class Grid:
    def __init__(self, width, height, depth, obstacles):
        self.width = width
        self.height = height
        self.depth = depth
        self.obstacles = obstacles
    
    def in_bounds(self, position):
        x, y, z = position
        return 0 <= x < self.width and 0 <= y < self.height and 0 <= z < self.depth
    
    def passable(self, position):
        return position not in self.obstacles
    
    def neighbors(self, position):
        x, y, z = position
        results = [(x+1, y, z), (x, y-1, z), (x, y, z-1), (x-1, y, z), (x, y+1, z), (x, y, z+1)]
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results
    
    def cost(self, current, next):
        return manhattan_distance(current, next)

