from collections import deque
import heapq
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

            
            state_key = (current_state.x, current_state.y, frozenset(current_state.collected_items))
            self.visited.add(state_key)

       
            for neighbor in current_state.get_neighbors(self.grid):
                neighbor_key = (neighbor.x, neighbor.y, frozenset(neighbor.collected_items))
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

            self.visited.add((current_state.x, current_state.y, frozenset(current_state.collected_items)))

            for neighbor in current_state.get_neighbors(self.grid):

                neighbor_signature = (neighbor.x, neighbor.y, frozenset(neighbor.collected_items))
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

        visited.add((state.x, state.y, frozenset(state.collected_items)))

        for neighbor in state.get_neighbors(self.grid):
            key = (neighbor.x, neighbor.y, frozenset(neighbor.collected_items))
            if key not in visited:
                result = self.recursive_dls(neighbor, limit - 1, visited, path + [neighbor])
                if result is not None:
                    return result

        visited.remove((state.x, state.y, frozenset(state.collected_items)))
        return None

