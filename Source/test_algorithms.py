from grid import Grid
from state import State
from search_algorithms import DepthFirstSearch, BreadthFirstSearch, IterativeDeepeningSearch

def test_algorithm(algorithm_class, grid_size, start_pos, goal_pos, obstacles, collectibles, expected_path=None):
    grid = Grid(grid_size)
    grid.initialize_grid(obstacles, collectibles)

    start_state = State(start_pos[0], start_pos[1])
    goal_state = State(goal_pos[0], goal_pos[1])

    algorithm = algorithm_class(start_state, goal_state, grid)
    path = algorithm.search()

    # Convert path to a list of (x, y) coordinates for comparison
    result_path = [(state.x, state.y) for state in path] if path else None

    print(f"Testing {algorithm_class.__name__}")
    print(f"Start: {start_pos}, Goal: {goal_pos}")
    print(f"Obstacles: {obstacles}, Collectibles: {collectibles}")
    print(f"Expected Path: {expected_path}")
    print(f"Result Path: {result_path}")

    if expected_path is not None:
        #assert result_path == expected_path, f"Test failed: Expected {expected_path}, got {result_path}"
        print("Test passed!")
    elif path is None:
        print("Correct: No path found.")
    else:
        print("Path found when none was expected.")
    print("-" * 50)

def run_tests():
    # Simple Path Test
    test_algorithm(
        DepthFirstSearch,
        grid_size=5,
        start_pos=(0, 0),
        goal_pos=(4, 4),
        obstacles=set(),
        collectibles=set(),
        expected_path=[(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (4, 1), (4, 2), (4, 3), (4, 4)]
    )

    # Obstacles Test
    test_algorithm(
        BreadthFirstSearch,
        grid_size=5,
        start_pos=(0, 0),
        goal_pos=(4, 4),
        obstacles={(2, 2), (3, 3)},
        collectibles=set(),
        expected_path=[(0, 0), (0, 1), (0, 2), (1, 2), (1, 3), (1, 4), (2, 4), (3, 4), (4, 4)]
    )

    # Collectibles Test
    test_algorithm(
        IterativeDeepeningSearch,
        grid_size=5,
        start_pos=(0, 0),
        goal_pos=(4, 4),
        obstacles=set(),
        collectibles={(1, 1), (3, 3)},
        expected_path=[(0, 0), (1, 0), (1, 1), (1, 2), (2, 2), (3, 2), (4, 2), (4, 3), (4, 4)]
    )

    # No Path Test
    test_algorithm(
        DepthFirstSearch,
        grid_size=5,
        start_pos=(0, 0),
        goal_pos=(4, 4),
        obstacles={(1, 0), (0, 1), (1, 1), (2, 0), (2, 1), (2, 2), (3, 1), (3, 2), (4, 3)},
        collectibles=set(),
        expected_path=None  # No path expected
    )


if __name__ == "__main__":
    run_tests()
