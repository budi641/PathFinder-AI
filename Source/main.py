from config import GRID_SIZE, OBSTACLE_POSITIONS, COLLECTIBLE_POSITIONS, START_POSITION, GOAL_POSITION, SEARCH_ALGORITHM
from grid import Grid
from state import State
from search_algorithms import DepthFirstSearch, BreadthFirstSearch, IterativeDeepeningSearch,Astar,GreedySearch,UniformCostSearch
from game import Game
import timeit
########
def select_search_algorithm(algorithm_name, start_state, goal_state, grid):
    match algorithm_name:
        case 'DFS':
            return DepthFirstSearch(start_state, goal_state, grid)
        case 'BFS':
            return BreadthFirstSearch(start_state, goal_state, grid)
        case 'IDS':
            return IterativeDeepeningSearch(start_state, goal_state, grid)
        case 'A*':
            return Astar(start_state, goal_state, grid)
        case 'GS':
            return GreedySearch(start_state, goal_state, grid)
        case 'UCS':
            return UniformCostSearch(start_state, goal_state, grid)
        case _:
            raise ValueError(f"Unknown search algorithm: {algorithm_name}")

def main():
    # Initialize the grid
    grid = Grid(size=GRID_SIZE)
    grid.initialize_grid(OBSTACLE_POSITIONS, COLLECTIBLE_POSITIONS)

    # Define start and goal states
    start_state = State(START_POSITION[0], START_POSITION[1])
    goal_state = State(GOAL_POSITION[0], GOAL_POSITION[1])

    # Select and run the search algorithm
    search_algorithm = select_search_algorithm(SEARCH_ALGORITHM, start_state, goal_state, grid)

    # Measure runtime
    start_time = timeit.default_timer()
    path = search_algorithm.search()
    end_time = timeit.default_timer()
    runtime = end_time - start_time

    # Display the results
    if path:
        print("Path found")

        for state in path:
            print(state)


        print(f"Runtime: {runtime:.6f} seconds")
    else:
        print("No path found.")

    # Visualize the path using the Game class
    game = Game(grid, start_state, goal_state, path)
    game.run()

if __name__ == "__main__":
    main()
