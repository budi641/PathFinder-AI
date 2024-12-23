from collections import deque
from queue import PriorityQueue
import heapq
import sys
import math
import random
from grid import *
from state import *

class DepthFirstSearch:
    def __init__(self, start_state, goal_state, grid):
        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid
        self.visited = set()
        self.stack = [(start_state, [start_state])] 
        self.search_tree = {}  # Dictionary to track the search tree

    def search(self):
        while self.stack:
            current_state, path = self.stack.pop()

           
            self.search_tree[current_state] = None if not path or len(path) < 2 else path[-2]

            if current_state.is_goal(self.goal_state):
                return path, self.search_tree  

            self.visited.add((current_state.x, current_state.y))


            for neighbor in current_state.get_neighbors_uninformed(self.grid):
                neighbor_key = (neighbor.x, neighbor.y)
                if neighbor_key not in self.visited:
                    self.stack.append((neighbor, path + [neighbor]))

        return None, self.search_tree 


class BreadthFirstSearch:
    def __init__(self, start_state, goal_state, grid):
        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid
        self.visited = set()
        self.queue = deque([(start_state, [start_state])])
        self.search_tree = {}

    def search(self):
        while self.queue:
            current_state, path = self.queue.popleft()
            self.search_tree[current_state] = None if not path or len(path) < 2 else path[-2]


            if current_state.is_goal(self.goal_state):
                return path, self.search_tree

            self.visited.add((current_state.x, current_state.y))

            for neighbor in current_state.get_neighbors_uninformed(self.grid):
                neighbor_signature = (neighbor.x, neighbor.y)
                if neighbor_signature not in self.visited:
                    self.queue.append((neighbor, path + [neighbor]))

        return None, self.search_tree

class IterativeDeepeningSearch:
    def __init__(self, start_state, goal_state, grid):
        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid
        self.max_depth = 100
        self.visited = set()
        self.stack = [(start_state, [start_state], 0)]
        self.search_tree = {}

    def search(self):
        for depth_limit in range(self.max_depth):
            self.stack = [(self.start_state, [self.start_state], 0)]
            self.visited = set()

            while self.stack:
                current_state, path, current_depth = self.stack.pop()
                self.search_tree[current_state] = None if not path or len(path) < 2 else path[-2]


                if current_state.is_goal(self.goal_state):
                    return path, self.search_tree

                if current_depth >= depth_limit:
                    continue

                self.visited.add((current_state.x, current_state.y))

                for neighbor in current_state.get_neighbors_uninformed(self.grid):
                    neighbor_key = (neighbor.x, neighbor.y)
                    if neighbor_key not in self.visited:
                        self.stack.append((neighbor, path + [neighbor], current_depth + 1))

        return None, self.search_tree

class GreedySearch:
    def __init__(self, start_state, goal_state, grid):
        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid
        self.visited = set()
        self.search_tree = {}
        self.queue = deque([(start_state, [start_state], grid.second_heuristic(start_state.get_neighbors(self.grid),goal_state))])  

    def search(self):
        while self.queue:


            current_state, path, neighbors_heuristic = self.queue.popleft()

            neighbors= current_state.get_neighbors(self.grid)
            
            if current_state.is_goal(self.goal_state):
                break

            self.visited.add((current_state.x, current_state.y))


            neighbors_heuristic = sorted(neighbors_heuristic, key=lambda item: item[1])

            
            for neighbor in neighbors_heuristic:

                neighbor_signature = (neighbor[0].x, neighbor[0].y)
                if neighbor_signature not in self.visited:
                   
                    
                    self.queue.append((neighbor[0], path + [neighbor[0]], self.grid.heuristic(neighbor[0].get_neighbors(self.grid),self.goal_state)))
                    self.search_tree[neighbor[0]] = current_state

                    break


        return path,self.search_tree

        
class Astar:
    def __init__(self, start_state, goal_state, grid):
        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid
        self.visited = set()
        self.queue = deque([(start_state, [start_state], grid.second_heuristic(start_state.get_neighbors(self.grid), goal_state))])
        self.search_tree = {}

    def search(self):
        while self.queue:
            current_state, path, neighbors_heuristic = self.queue.popleft()
            neighbors = current_state.get_neighbors(self.grid)

            if current_state.is_goal(self.goal_state):
                return path, self.search_tree

            self.visited.add((current_state.x, current_state.y))

            astar_heuristic = []

            for neighbor in neighbors_heuristic:
                astar_heuristic.append([neighbor[0], neighbor[1] + self.grid.get_distance(neighbor[0], self.goal_state)])

            astar_heuristic = sorted(astar_heuristic, key=lambda item: item[1])

            for neighbor in astar_heuristic:
                neighbor_signature = (neighbor[0].x, neighbor[0].y)
                if neighbor_signature not in self.visited:
                    self.search_tree[neighbor[0]] = current_state
                    self.queue.append((neighbor[0], path + [neighbor[0]], self.grid.heuristic(neighbor[0].get_neighbors(self.grid), self.goal_state)))

        return None, self.search_tree




class UniformCostSearch:
    def __init__(self, start_state, goal_state, grid):
        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid
        self.visited = set()
        self.queue = PriorityQueue()
        self.counter = 0
        self.queue.put((0, self.counter, start_state, [start_state]))
        self.search_tree = {}

    def calculate_cost(self, state, current_cost):
        cost = current_cost + 5  

        if self.grid.is_collectible(state.x, state.y):
            cost -= 4 

        if self.grid.is_obstacle(state.x, state.y):
            cost = sys.maxsize

        return cost

    def search(self):
        while not self.queue.empty():
            current_cost, _, current_state, path = self.queue.get()

            if current_state.is_goal(self.goal_state):
                return path, self.search_tree

            neighbor_signature = (current_state.x, current_state.y)
            if neighbor_signature not in self.visited:
                self.visited.add(neighbor_signature)

                for neighbor in current_state.get_neighbors_uninformed(self.grid):
                    neighbor_id = (neighbor.x, neighbor.y)
                    
                    if neighbor_id not in self.visited:
                        new_cost = self.calculate_cost(neighbor, current_cost)
                        if new_cost != sys.maxsize:
                            self.counter += 1
                            self.search_tree[neighbor] = current_state
                            self.queue.put((new_cost, self.counter, neighbor, path + [neighbor]))

        return None, self.search_tree

    
class HillClimbing:
    def __init__(self, start_state, goal_state, grid):
        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid
        self.search_tree = {}

    def search(self):
        current_state = self.start_state
        path = [current_state]

        while not current_state.is_goal(self.goal_state):

            neighbors = current_state.get_neighbors_uninformed(self.grid)

            best_neighbor = min(neighbors, key=lambda neighbor: self.grid.get_distance(neighbor, self.goal_state))

            if self.grid.get_distance(best_neighbor, self.goal_state) >= self.grid.get_distance(current_state, self.goal_state):
                break

            self.search_tree[best_neighbor] = current_state
            current_state = best_neighbor
            path.append(current_state)

        return path if current_state.is_goal(self.goal_state) else None, self.search_tree


class SimulatedAnnealing:
    def __init__(self, start_state, goal_state, grid, initial_temperature=1000, cooling_rate=0.99):
        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid
        self.temperature = initial_temperature
        self.cooling_rate = cooling_rate
        self.search_tree = {}

    def search(self):
        current_state = self.start_state
        path = [current_state]

        while self.temperature > 1:

            neighbors = current_state.get_neighbors_uninformed(self.grid)
            if not neighbors:
                break

            if(current_state == self.goal_state):
                self.temperature = 0
                break

            next_state = random.choice(neighbors)

            current_cost = self.grid.get_distance(current_state, self.goal_state)
            next_cost = self.grid.get_distance(next_state, self.goal_state)
            delta_cost = next_cost - current_cost

            if delta_cost < 0 or math.exp(-delta_cost / self.temperature) > random.random():
                self.search_tree[next_state] = current_state
                current_state = next_state
                path.append(current_state)

            self.temperature *= self.cooling_rate
        
        return path, self.search_tree





