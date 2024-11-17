import networkx as nx
import matplotlib.pyplot as plt
from search_algorithms import DepthFirstSearch, BreadthFirstSearch, IterativeDeepeningSearch,Astar,GreedySearch,UniformCostSearch,HillClimbing,SimulatedAnnealing
from config import GRID_SIZE, OBSTACLE_POSITIONS, COLLECTIBLE_POSITIONS, START_POSITION, GOAL_POSITION, SEARCH_ALGORITHM


def visualize_search_tree(search_tree, start_state, goal_state):
    G = nx.DiGraph()

    for child, parent in search_tree.items():
        G.add_node(child, label=f"({child.x}, {child.y}): {len(child.collected_items)}")
        if parent is not None:
            G.add_edge(parent, child)



    pos = nx.spring_layout(G)
    

    node_colors = []
    for node in G.nodes:
        if node == start_state:
            node_colors.append("green") 
        elif node == goal_state:
            node_colors.append("red") 
        else:
            node_colors.append("skyblue")  

    labels = {node: f"({node.x}, {node.y}): {len(node.collected_items)}" for node in G.nodes}
    nx.draw(G, pos, labels=labels, with_labels=True, node_color=node_colors, node_size=500, font_size=5, font_color="black", font_weight="bold")

    plt.show()






