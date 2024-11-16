class State:
    def __init__(self, x, y, collected_items=None):
        self.x = x
        self.y = y
        # Track collected items to maximize score
        self.collected_items = collected_items if collected_items else set()

    def get_neighbors(self, grid):
        # Only non-diagonal moves: up, down, left, right
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []

        for dx, dy in directions:
            nx, ny = self.x + dx, self.y + dy
            if grid.is_within_bounds(nx, ny):
                # Collect items if moving to a collectible cell
                new_collected = self.collected_items.copy()
                if grid.is_collectible(nx, ny):
                    new_collected.add((nx, ny))
                neighbors.append(State(nx, ny, new_collected))

        return neighbors
    
    def get_neighbors_uninformed(self, grid):
        # Only non-diagonal moves: up, down, left, right
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []
        for dx, dy in directions:
            nx, ny = self.x + dx, self.y + dy
            if grid.is_within_bounds(nx, ny) and not grid.is_obstacle(nx, ny):
                # Collect items if moving to a collectible cell
                new_collected = self.collected_items.copy()
                if grid.is_collectible(nx, ny):
                    new_collected.add((nx, ny))
                neighbors.append(State(nx, ny, new_collected))
        return neighbors

    
        
    def is_goal(self, goal_state):
        return self.x == goal_state.x and self.y == goal_state.y

    def __repr__(self):
        return f"State(x={self.x}, y={self.y}, collected_items={self.collected_items})"


