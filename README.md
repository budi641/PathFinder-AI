# **Pathfinding AI Project**

### **Overview**

This project implements a pathfinding AI capable of navigating a 2D grid to reach a goal position while collecting items and avoiding obstacles. The AI will use a combination of search algorithms to find optimal paths, focusing on maximizing collectibles while reaching the goal.

---

## **1. Problem Modeling**

### **Problem Overview**

#### **State Space**
- The state space is represented by an \( n \times n \) grid, where each cell has one of three types:
  - **Collectible Cells**: Cells that contain an item the AI can collect.
  - **Obstacle Cells**: Impassable cells that the AI must avoid.
  - **Empty Cells**: Passable cells that do not contain collectibles.
- Each state consists of:
  - The current position of the AI on the grid (e.g., coordinates `(x, y)`).
  - A record of collected items to maximize the AI’s collectible count on its path to the goal.

#### **Action Space**
- **Available Actions**: The AI can move in the four cardinal directions—up, down, left, and right.
- **Movement Rules**:
  - The AI cannot move diagonally.
  - Movement into an obstacle cell is not allowed.
  - Moving into a collectible cell results in collecting the item.

---

### **Modeling Assumptions**

1. **Grid Size**: The grid is square with dimensions \( n \times n \).
2. **Obstacles and Collectibles**: Positions of obstacles and collectibles are fixed and known at the start.
3. **Static Environment**:
   - The grid layout, including obstacle and collectible locations, does not change during the AI’s navigation.
4. **Single Objective Pathfinding**:
   - The AI’s goal is to collect the maximum number of collectibles while avoiding obstacles and reaching the goal cell.
5. **Path Cost**:
   - Each move has a uniform cost, so path length is not weighted; rather, priority is on reaching the goal and maximizing collectible count.

---

### **Initial and Goal States**

- **Initial State**:
  - Example: The AI begins at the top-left corner of the grid, at coordinates `(0, 0)`.
  - At the initial state, the AI has not collected any items.

- **Goal State**:
  - Example: The goal cell could be at the bottom-right corner, such as `(n-1, n-1)`.
  - The goal is achieved once the AI reaches this cell, with the objective to have collected as many items as possible on the way.

---

### **Environmental Assumptions**

- **Static Obstacles**: Obstacles do not move or change throughout the AI’s navigation.
- **Fixed Grid Size**: The grid size remains consistent, with no cells added or removed during runtime.
- **No Diagonal Movement**: The AI is restricted to horizontal and vertical moves only, enhancing predictability and alignment with problem constraints.

---

## **2. Code Structure**

### **Grid Class** (`grid.py`)
- Manages the grid layout, obstacles, collectibles, and provides helper functions for cell checks, grid size validation, action validation, and estimating the state space size.

```python
class Grid:
    def __init__(self, size, obstacles=None, collectibles=None):
        self.size = size
        self.obstacles = obstacles if obstacles else set()
        self.collectibles = collectibles if collectibles else set()

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
        total_cells = self.size * self.size
        valid_states = total_cells - len(self.obstacles)
        collectible_combinations = 2 ** len(self.collectibles)
        state_space_size = valid_states * collectible_combinations
        return state_space_size
```

#### **Explanation**:
- **is_within_bounds(x, y)**: Ensures moves stay within grid boundaries.
- **is_valid_grid_size(size)**: Checks that the grid size is within a set limit.
- **validate_action(x, y, grid_size)**: Validates if an action will stay within grid bounds.
- **estimate_state_space()**: Estimates the size of the state space based on grid cells, obstacles, and collectible combinations.

### **State Class** (`state.py`)
- Represents a single cell in the grid and tracks the AI’s current position and collected items.

```python
class State:
    def __init__(self, x, y, collected_items=None):
        self.x = x
        self.y = y
        self.collected_items = collected_items if collected_items else set()

    def get_neighbors(self, grid):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []

        for dx, dy in directions:
            nx, ny = self.x + dx, self.y + dy
            if grid.is_within_bounds(nx, ny) and not grid.is_obstacle(nx, ny):
                new_collected = self.collected_items.copy()
                if grid.is_collectible(nx, ny):
                    new_collected.add((nx, ny))
                neighbors.append(State(nx, ny, new_collected))

        return neighbors

    def is_goal(self, goal_state):
        return self.x == goal_state.x and self.y == goal_state.y

    def __repr__(self):
        return f"State(x={self.x}, y={self.y}, collected_items={self.collected_items})"
```

#### **Explanation**:
- **get_neighbors(grid)**: Generates potential moves, checking boundaries, obstacles, and collectibles.
- **is_goal(goal_state)**: Checks if the AI has reached the goal.
- **collected_items**: Tracks items collected by the AI in the current state.
---

## **3. Problem Size Estimation**

- **Method**: The size of the state space is estimated by considering the number of valid cells and collectible configurations.
  - **Valid Positions**: The total number of cells in the grid minus the number of obstacle cells.
  - **Collectible Combinations**: For each collectible, there are two possible states: collected or not collected. This results in `2^(number of collectibles)` combinations.
  - **Total State Space**: The product of valid cells and collectible combinations.

**Example:**

For a 5x5 grid with 3 obstacles and 2 collectibles:
- Total cells = 25  
- Obstacles = 3  
- Valid cells = 25 - 3 = 22  
- Collectible combinations = 2^2 = 4  
- State space size = 22 * 4 = 88  

--- 


**Performance and Visualisation**

https://docs.google.com/document/d/1iCnu9sq-GrP05LBeqpqgcEWXa0k-b7v0AVj7XQ5SLEI/edit?usp=sharing
