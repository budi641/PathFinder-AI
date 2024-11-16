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

    def search(self):
        while self.stack:
            current_state, path = self.stack.pop()

            if current_state.is_goal(self.goal_state):
                return path  

            
       
            self.visited.add((current_state.x, current_state.y))

        
            for neighbor in current_state.get_neighbors_uninformed(self.grid):
                neighbor_key = (neighbor.x, neighbor.y)
                if neighbor_key not in self.visited:
                   
                    self.stack.append((neighbor, path + [neighbor]))

        return None  

class BreadthFirstSearch:
    def __init__(self, start_state, goal_state, grid):

        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid
        self.visited = set()  # To track visited states
        self.queue = deque([(start_state, [start_state])])  # Queue stores (state, path_to_state)

    def search(self):

        while self.queue:
            current_state, path = self.queue.popleft()

            if current_state.is_goal(self.goal_state):
                return path

            self.visited.add((current_state.x, current_state.y))

            for neighbor in current_state.get_neighbors_uninformed(self.grid):

                neighbor_signature = (neighbor.x, neighbor.y)
                if neighbor_signature not in self.visited:
                    self.queue.append((neighbor, path + [neighbor]))


        return None

class IterativeDeepeningSearch:
    def __init__(self, start_state, goal_state, grid):
        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid

    def search(self):
        depth = 0
        while True:
            result = self.depth_limited_search(self.start_state, depth)
            if result is not None:
                return result
            depth += 1

    def depth_limited_search(self, state, limit):
        return self.recursive_dls(state, limit, set(), [state])

    def recursive_dls(self, state, limit, visited, path):
        if state.is_goal(self.goal_state):
            return path

        if limit == 0:
            return None

        visited.add((state.x, state.y))

        for neighbor in state.get_neighbors_uninformed(self.grid):
            key = (neighbor.x, neighbor.y)
            if key not in visited:
                result = self.recursive_dls(neighbor, limit - 1, visited, path + [neighbor])
                if result is not None:
                    return result

        visited.remove((state.x, state.y))
        return None

class GreedySearch:
    def __init__(self, start_state, goal_state, grid):
        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid
        self.visited = set()
        self.queue = deque([(start_state, [start_state], grid.heuristic(start_state.get_neighbors(self.grid),goal_state))])  

    def search(self):
        while self.queue:


            current_state, path, neighbors_heuristic = self.queue.popleft()

            neighbors= current_state.get_neighbors(self.grid)
            
            if current_state.is_goal(self.goal_state):
                return path

            self.visited.add((current_state.x, current_state.y))


            neighbors_heuristic = sorted(neighbors_heuristic, key=lambda item: item[1])

            
            for neighbor in neighbors_heuristic:

                neighbor_signature = (neighbor[0].x, neighbor[0].y)
                if neighbor_signature not in self.visited:
                   
                    self.queue.append((neighbor[0], path + [neighbor[0]], self.grid.heuristic(neighbor[0].get_neighbors(self.grid),self.goal_state)))
                    break


        return None
        
class Astar:
    def __init__(self, start_state, goal_state, grid):
        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid
        self.visited = set()
        self.queue = deque([(start_state, [start_state], grid.heuristic(start_state.get_neighbors(self.grid),goal_state))])  

    def search(self):
        while self.queue:

            current_state, path, neighbors_heuristic = self.queue.popleft()

            neighbors= current_state.get_neighbors(self.grid)

            if current_state.is_goal(self.goal_state):
                return path

            self.visited.add((current_state.x, current_state.y))

            astar_heuristic = []

            

            for neighbor in neighbors_heuristic:
               
               astar_heuristic.append([neighbor[0] , neighbor[1] + self.grid.get_distance(neighbor[0], self.goal_state)])

            astar_heuristic = sorted(astar_heuristic, key=lambda item: item[1])
            #print( astar_heuristic)
            for neighbor in astar_heuristic:

                neighbor_signature = (neighbor[0].x, neighbor[0].y)
                if neighbor_signature not in self.visited:
                    
                    self.queue.append((neighbor[0], path + [neighbor[0]], self.grid.heuristic(neighbor[0].get_neighbors(self.grid),self.goal_state)))
                    break


        return None




class UniformCostSearch:
    def __init__(self, start_state, goal_state, grid):
        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid
        self.visited = set()
        self.queue = PriorityQueue()
        self.queue.put((0, start_state, [start_state]))

    def search(self):
        while not self.queue.empty():
            current_cost, current_state, path = self.queue.get()

            if current_state.is_goal(self.goal_state):
                return path, current_cost

            
            self.visited.add(current_state.x, current_state.y)

            for neighbor in current_state.get_neighbors_uninformed(self.grid):
                neighbor_key = (neighbor.x, neighbor.y)
                if neighbor_key not in self.visited:
                    new_cost = current_cost + 1
                    self.queue.put((new_cost, neighbor_key, path + [neighbor]))

        return None


class HillClimbing:
    def __init__(self, start_state, goal_state, grid):
        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid

    def search(self):
        current_state = self.start_state
        path = [current_state]

        while not current_state.is_goal(self.goal_state):

            neighbors = current_state.get_neighbors_uninformed(self.grid)


            best_neighbor = min(neighbors, key=lambda neighbor: self.grid.get_distance(neighbor, self.goal_state))

            if self.grid.get_distance(best_neighbor, self.goal_state) >= self.grid.get_distance(current_state, self.goal_state):
                break

            current_state = best_neighbor
            path.append(current_state)

        return path if current_state.is_goal(self.goal_state) else None


class SimulatedAnnealing:
    def __init__(self, start_state, goal_state, grid, initial_temperature=1000, cooling_rate=0.99):
        self.start_state = start_state
        self.goal_state = goal_state
        self.grid = grid
        self.temperature = initial_temperature
        self.cooling_rate = cooling_rate

    def search(self):
        current_state = self.start_state
        path = [current_state]

        while self.temperature > 1:

            neighbors = current_state.get_neighbors_uninformed(self.grid)
            if not neighbors:
                break

            if(current_state == self.goal_state):
                break

            next_state = random.choice(neighbors)

            # Calculate energy 
            current_cost = self.grid.get_distance(current_state, self.goal_state)
            next_cost = self.grid.get_distance(next_state, self.goal_state)
            delta_cost = next_cost - current_cost

            # next state 
            if delta_cost < 0 or math.exp(-delta_cost / self.temperature) > random.random():
                current_state = next_state
                path.append(current_state)

            # Cool down 
            self.temperature *= self.cooling_rate
        
        
        return path
            




