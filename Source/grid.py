import math
import sys
class Grid:
    def __init__(self, size, obstacles=None, collectibles=None):
        self.size = size
        self.obstacles = obstacles if obstacles else set()
        self.collectibles = collectibles if collectibles else set()
    def get_size(self):
        return self.size
    def is_within_bounds(self, x, y):
        return 0 <= x < self.size and 0 <= y < self.size

    def is_valid_grid_size(size, max_size=100):

        return 1 <= size <= max_size

    def validate_action(x, y, grid_size):

        return 0 <= x < grid_size and 0 <= y < grid_size


    def is_obstacle(self, x, y):
        return (x, y) in self.obstacles

    def is_collectible(self, x, y):
        return (x, y) in self.collectibles


    def initialize_grid(self, obstacle_positions, collectible_positions):
        self.obstacles.update(obstacle_positions)
        self.collectibles.update(collectible_positions)

    def estimate_state_space(self):
        # Total number of cells in the grid
        total_cells = self.size * self.size

        # Subtract the number of obstacles to get the valid, walkable states
        valid_states = total_cells - len(self.obstacles)

        # For each collectible, the AI can either have it or not, adding a factor to the state space
        collectible_combinations = 2 ** len(self.collectibles)

        # The total state space size is the product of valid positions and collectible combinations
        state_space_size = valid_states * collectible_combinations
        return state_space_size
    def heuristic (self, neighbors, goal_state):

      neighbors_heuristic=[]
      for neighbor in (neighbors):
        h=5
        if neighbor.is_goal(goal_state):
             h=0
        if self.is_obstacle(neighbor.x, neighbor.y):
             h=sys.maxsize

        if self.is_collectible(neighbor.x, neighbor.y):
              h=1


        neighbors_heuristic.append([neighbor,h])

      return neighbors_heuristic
   
    def second_heuristic(self, neighbors, goal_state):
       neighbors_heuristic=[]
       for neighbor in neighbors: 
        h = abs(neighbor.x - goal_state.x) + abs(neighbor.y - goal_state.y)
        neighbors_heuristic.append([neighbor,h])
        
        return neighbors_heuristic
        
    def get_distance(self, neighbor, goal_state):

          distance = math.sqrt((neighbor.x - goal_state.x)**2 + (neighbor.y - goal_state.y)**2)

          return distance

