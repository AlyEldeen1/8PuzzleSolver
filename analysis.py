import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from backend import solver, puzzle

def run_analysis():

    puzzle_instance = puzzle([])

    results = defaultdict(dict)

    while True: #infinite loop till it found a solvable inital state
        initial_state = puzzle_instance.randomize_initial_state()
        if puzzle_instance.is_solvable(initial_state):
            break  

    # Display the initial state
    print("Using solvable initial state:", initial_state)

    puzzle_solver = solver(puzzle_instance)

    # Test BFS
    bfs_solution = puzzle_solver.bfs(initial_state)
    results['BFS'] = bfs_solution if bfs_solution else "No solution found"

    # Test DFS
    dfs_solution = puzzle_solver.dfs(initial_state)
    results['DFS'] = dfs_solution if dfs_solution else "No solution found"

    # Test IDS
    ids_solution = puzzle_solver.ids(initial_state)
    results['IDS'] = ids_solution if ids_solution else "No solution found"

    # Test A* with Manhattan heuristic
    astar_solution_manhattan = puzzle_solver.Astar(initial_state, heuristic="manhattan")
    results['A* (Manhattan)'] = astar_solution_manhattan if astar_solution_manhattan else "No solution found"

    # Test A* with Euclidean heuristic
    astar_solution_euclidean = puzzle_solver.Astar(initial_state, heuristic="euclidean")
    results['A* (Euclidean)'] = astar_solution_euclidean if astar_solution_euclidean else "No solution found"

    algorithms = []
    nodes_expanded = []
    search_depth = []
    runtime = []

    for key, value in results.items():
        if isinstance(value, dict):
            algorithms.append(key)
            nodes_expanded.append(value["nodes_expanded"])
            search_depth.append(value["search_depth"])
            runtime.append(value["runtime"])
        else:
            algorithms.append(key)
            nodes_expanded.append(0) 
            search_depth.append(0)  
            runtime.append(0)  

    # Generate a graphical report using matplotlib
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))

    # Nodes expanded
    axs[0].bar(algorithms, nodes_expanded, color=['blue', 'orange', 'green', 'red'])
    axs[0].set_title('Nodes Expanded')
    axs[0].set_ylabel('Number of Nodes Expanded')
    axs[0].set_xlabel('Algorithm')

    # Search depth
    axs[1].bar(algorithms, search_depth, color=['blue', 'orange', 'green', 'red'])
    axs[1].set_title('Search Depth')
    axs[1].set_ylabel('Depth')
    axs[1].set_xlabel('Algorithm')

    # Runtime
    axs[2].bar(algorithms, runtime, color=['blue', 'orange', 'green', 'red'])
    axs[2].set_title('Runtime (seconds)')
    axs[2].set_ylabel('Time')
    axs[2].set_xlabel('Algorithm')

    # Add initial state to the top of the report
    initial_state_str = "\n".join([" ".join(map(str, row)) for row in np.array(initial_state).reshape(3, 3)])
    fig.text(0.5, 0.95, 'Initial State:\n' + initial_state_str, ha='center', va='top', fontsize=12)

    
    plt.savefig('puzzle_analysis_report.png')
    plt.close(fig)

# Run the analysis
run_analysis()