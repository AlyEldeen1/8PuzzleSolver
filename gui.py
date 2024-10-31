import tkinter as tk
from tkinter import messagebox
import random
from backend import node, puzzle, solver 

class PuzzleGUI:
    def __init__(self, master):
        self.master = master
        master.title("8-Puzzle Solver")

        self.puzzle_instance = puzzle([])
        self.initial_state = [1,2,3,4,0,5,7,8,6]
        self.solver_instance = solver(self.puzzle_instance)

        self.create_widgets()
        self.update_grid(self.initial_state)

    def create_widgets(self):
        self.grid_frame = tk.Frame(self.master)
        self.grid_frame.pack()

        self.buttons = []
        for i in range(3):
            row_buttons = []
            for j in range(3):
                btn = tk.Button(self.grid_frame, text="", font=("Arial", 24), width=5, height=2)
                btn.grid(row=i, column=j)
                row_buttons.append(btn)
            self.buttons.append(row_buttons)

        self.algorithm_label = tk.Label(self.master, text="Select Search Algorithm:")
        self.algorithm_label.pack()

        self.algorithm_var = tk.StringVar(value="BFS")
        self.algorithm_menu = tk.OptionMenu(self.master, self.algorithm_var, "BFS", "DFS", "IDS", "A* (Manhattan)", "A* (Euclidean)")
        self.algorithm_menu.pack()

        self.solve_button = tk.Button(self.master, text="Solve", command=self.solve_puzzle)
        self.solve_button.pack()

        # Label to display the number of moves
        self.move_count_label = tk.Label(self.master, text="Moves: 0")
        self.move_count_label.pack()

        # Button to shuffle the puzzle
        self.shuffle_button = tk.Button(self.master, text="Shuffle", command=self.shuffle_puzzle)
        self.shuffle_button.pack()

        # Button to return to initial state
        self.return_button = tk.Button(self.master, text="Return", command=self.return_to_initial_state)
        self.return_button.pack()

        # Check if the puzzle is already solved and disable the solve button if true
        self.update_solve_button_state()

    def update_grid(self, state):
        for i in range(3):
            for j in range(3):
                value = state[i * 3 + j]
                self.buttons[i][j].config(text=str(value) if value != 0 else "", bg="lightgray")

    def solve_puzzle(self):
        # Check if the puzzle is already in the goal state
        if self.puzzle_instance.is_goal(self.initial_state):
            messagebox.showinfo("Info", "Puzzle already solved.")
            return 

        selected_algorithm = self.algorithm_var.get()
        solution = None

        if selected_algorithm == "BFS":
            solution = self.solver_instance.bfs(self.initial_state)
        elif selected_algorithm == "DFS":
            solution = self.solver_instance.dfs(self.initial_state)
        elif selected_algorithm == "IDS":
            solution = self.solver_instance.ids(self.initial_state)
        elif selected_algorithm == "A* (Manhattan)":
            solution = self.solver_instance.Astar(self.initial_state, heuristic="manhattan")
        elif selected_algorithm == "A* (Euclidean)":
            solution = self.solver_instance.Astar(self.initial_state, heuristic="euclidean")

        if solution:
            path = solution["path"]
            self.display_solution(path)
            self.move_count_label.config(text=f"Moves: {len(path) - 1}")  
        else:
            messagebox.showinfo("Result", "No solution found.")

        self.update_solve_button_state()

    def shuffle_puzzle(self):
        """Shuffle the puzzle and update the grid."""
        self.initial_state = self.puzzle_instance.randomize_initial_state()
        self.update_grid(self.initial_state)
        self.move_count_label.config(text="Moves: 0")  # Reset the move count display

        self.solve_button.config(state=tk.NORMAL)

    def return_to_initial_state(self):
        """Return to the initial state of the puzzle."""
        self.update_grid(self.initial_state)
        self.move_count_label.config(text="Moves: 0")  # Reset the move count display

        self.update_solve_button_state()

    def update_solve_button_state(self):
        if self.puzzle_instance.is_goal(self.initial_state):
            self.solve_button.config(state=tk.DISABLED)
        else:
            self.solve_button.config(state=tk.NORMAL)

    def display_solution(self, path):
        for step in path:
            self.update_grid(step)  # Update grid to show the current state
            self.master.update()  # Refresh the GUI
            self.master.after(500)  # Pause for half a second to visualize the move

def main():
    root = tk.Tk()
    gui = PuzzleGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()