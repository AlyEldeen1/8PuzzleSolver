# 8-Puzzle Solver

## Overview

The 8-Puzzle Solver is a Python project that implements various search algorithms to solve the classic 8-puzzle problem. This project demonstrates the effectiveness of different algorithms, including Depth-First Search (DFS), Breadth-First Search (BFS), Iterative Deepening DFS, and A* Search, using both Manhattan and Euclidean heuristics.

## Features

- Solves the 8-puzzle problem using multiple search strategies.
- Implements Depth-First Search (DFS), Breadth-First Search (BFS), Iterative Deepening DFS, and A* algorithms.
- Compares the performance of each algorithm based on the number of nodes expanded.
- Provides detailed traceability of each step in the puzzle solution.
- Visualizes the solution process (bonus feature).

## Getting Started

### Prerequisites

- Python 

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/AlyEldeen1/8PuzzleSolver
   cd repo-name

Algorithms
Depth-First Search (DFS)
The DFS algorithm explores as far as possible along each branch before backtracking. It is implemented to find a solution for the 8-puzzle problem, but may not guarantee the shortest path due to its nature of exploration.

Breadth-First Search (BFS)
The BFS algorithm explores all neighbor nodes at the present depth prior to moving on to nodes at the next depth level. This approach guarantees the shortest solution path but may consume a lot of memory.

Iterative Deepening DFS
This method combines the space efficiency of DFS with the optimality of BFS. It repeatedly performs DFS with increasing depth limits until a solution is found.

A* Search
A* is a popular pathfinding algorithm that uses heuristics to improve search efficiency. This implementation includes both Manhattan and Euclidean heuristics, allowing for comparison of their effectiveness in solving the 8-puzzle.

Results
The project compares the performance of each algorithm based on the number of nodes expanded during the search process. The results demonstrate that while DFS is fast, it often expands significantly more nodes than BFS and A*.
