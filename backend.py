import numpy as np               
from collections import deque     
import heapq                      
import math                       
import random
import time

class node:
    # this is node class that represent a node in the search method
  
    def __init__(self,state, parent,cost,her):
        
        self.state = state
        self.parent = parent
        self.cost = cost
        self.her = her      #(heuristic value for A*)
    
    def total_cost(self):
        return self.cost + self.her 
    def trace_path(self):
        current = self
        path = []
        while current:
            path.append(current.state)
            current= current.parent
            
        path.reverse()
        return path
     
    def __lt__(self, other): #for the lessthan comparison in piriority queue
        return self.total_cost() < other.total_cost()  

class puzzle:
    
    #puzzle class for propertites of 8 puzzle and its states

    def __init__(self, initial_state):
        self.initial_state= initial_state
        self.goal= [0,1, 2, 3, 4, 5, 6, 7, 8]

    def randomize_initial_state(self):
        state = self.goal[:]
        while True:
            random.shuffle(state)
            if self.is_solvable(state):
                return state
            
    def is_solvable(self,state): # to know if its solvable or not based on number of inversions
        inversions = 0
        for i in range(len(state)):
            for j in range(i+1,len(state)):
                if state[i] != 0 and state[j] !=0 and state[i] > state[j]:
                    inversions = inversions + 1
        if(inversions%2==0):
            return True
        else:
            return "Puzzle isn't solvable"

    def is_goal(self,state):
        return state== self.goal
    
    def get_possible_moves(self,state):
        zeroindex = state.index(0)
        possiblemoves = []
        row, col = divmod(zeroindex, 3) # function for getting row and col from a 1D list

        if (row>0):
            possiblemoves.append("up")
        if(row<2):
            possiblemoves.append("down")
        if(col>0):
            possiblemoves.append("left")
        if(col<2):
            possiblemoves.append("right")

        return possiblemoves
    
    def move(self,state,direction): 
        zeroindex = state.index(0)
        newstate =state[:]
        if(direction=="up"):
            new_index = zeroindex - 3
        elif(direction == "down"):
            new_index = zeroindex + 3
        elif(direction == "right"):
            new_index = zeroindex + 1
        elif(direction == "left"):
            new_index =  zeroindex - 1

        newstate[zeroindex] , newstate[new_index] = newstate[new_index] , newstate[zeroindex]
        return newstate
        
class solver:
    def __init__ (self,puzzle):
        self.puzzle = puzzle


    def expand_node(self,currentnode,searchtype,heurstictype):
       
        neighbors = []
        validmoves = self.puzzle.get_possible_moves(currentnode.state)

        for move in validmoves:
            newstate = self.puzzle.move(currentnode.state,move)

            if searchtype != "DFS" and searchtype != "BFS" and searchtype != "IDS": #for A*

                if heurstictype == "Manhattan":

                    heursticvalue = self.Manhattan_heuristic(newstate)
                    newnode = node(newstate,currentnode,currentnode.cost+1,heursticvalue)

                else:

                    heursticvalue = self.Euclidean_heuristic(newstate)
                    newnode = node(newstate,currentnode,currentnode.cost+1,heursticvalue)                
            else:

                newnode = node(newstate,currentnode,currentnode.cost + 1,0)

            neighbors.append(newnode)

        return neighbors
    

    def Manhattan_heuristic(self,state):
        totaldistance = 0

        for tile, value in enumerate(state):
            if value != 0:
                goalindex = value

                currentrow, currentcol = divmod(tile,3) # get current tile row and col
                goalrow,goalcol = divmod(goalindex,3) # get the goal tile row and col
                #manhattan distance calculation :
                distance = abs(currentrow - goalrow) + abs(currentcol - goalcol)
                totaldistance += distance

        return totaldistance
    
    def Euclidean_heuristic(self,state):
        totaldistance = 0
        
        for tile, value in enumerate(state):
            if value != 0:
                goalindex = value
                
                currentrow, currentcol = divmod(tile,3)
                goalrow,goalcol = divmod(goalindex,3)
                #euclidean distance calculation : 
                distance = math.sqrt((currentrow - goalrow) ** 2 + (currentcol - goalcol) ** 2)
                totaldistance += distance
        return totaldistance
    
    def bfs(self, state ):
            #FIFO
        start_time = time.time()
        root= node(state,None,0,0)
        frontier= deque([root])
        explored=set()
        frontier_states = set() # this set to increase performance
        nodesExpanded = 0


        while len(frontier)!=0:
            currentnode= frontier.popleft()
            current_state_tuple = tuple(currentnode.state) #Tuple to make states immutable
            explored.add(current_state_tuple) 
            nodesExpanded = nodesExpanded + 1

            if self.puzzle.is_goal(currentnode.state):
                runtime = time.time() - start_time
                path = currentnode.trace_path()
                return {
                        "path": path,
                        "cost": currentnode.cost,
                        "nodes_expanded": nodesExpanded,
                        "search_depth": len(path) - 1,
                        "runtime": runtime
                        }
            
            else:
                for neighbor in self.expand_node(currentnode,"BFS",None):
                    neighbor_state_tuple = tuple(neighbor.state)
                    if neighbor_state_tuple not in explored and neighbor_state_tuple not in frontier_states:
                        frontier.append(neighbor)
                        frontier_states.add(neighbor_state_tuple)

                if current_state_tuple in frontier_states:
                    frontier_states.remove(current_state_tuple)

        return None
    
    def dfs(self, state):
        #LIFO
        start_time = time.time()
        root = node(state, None, 0, 0)
        frontier = [root]
        explored = set()
        frontier_states = set()
        nodesExpanded = 0

        while len(frontier) != 0:
            current_node = frontier.pop()

            if self.puzzle.is_goal(current_node.state):
                runtime = time.time() - start_time
                path = current_node.trace_path()
                return {
                        "path": path,
                        "cost": current_node.cost,
                        "nodes_expanded": nodesExpanded,
                        "search_depth": len(path) - 1,
                        "runtime": runtime
                        }
            
            current_state_tuple = tuple(current_node.state)
            explored.add(current_state_tuple)
            nodesExpanded = nodesExpanded + 1
            

            for neighbor in self.expand_node(current_node, "DFS", None):
                neighbor_state_tuple = tuple(neighbor.state)

                if neighbor_state_tuple not in explored and neighbor_state_tuple not in frontier_states:
                    frontier.append(neighbor)
                    frontier_states.add(neighbor_state_tuple)

            if current_state_tuple in frontier_states:
                frontier_states.remove(current_state_tuple)
        return None

    
    def display_path(self, path):
        for state in path:
            for i in range(0, 9, 3):
                print(state[i:i+3])
            print("\n")

    def dls(self,state,limit,nodesExpanded):
        #depth limit search helper function for IDS
        root= node(state, None, 0, 0)
        frontier= [(root,0)]
        explored= set()
        frontier_states = set()

        while len(frontier)!=0:

            currentnode,depth =frontier.pop()
            if self.puzzle.is_goal(currentnode.state):
                path = currentnode.trace_path()
                return {
                        "path": path,
                        "cost": currentnode.cost,
                        "nodes_expanded": nodesExpanded,
                        "search_depth": depth,
                        "runtime": None
                        }
            
            current_state_tuple = tuple(currentnode.state)

            if depth < limit:
                explored.add(current_state_tuple)
                nodesExpanded = nodesExpanded + 1

                for neighbor in self.expand_node(currentnode, "IDS",None):
                    neighbor_state_tuple = tuple(neighbor.state)
                    if neighbor_state_tuple not in explored and neighbor_state_tuple not in frontier_states:
                        frontier.append((neighbor,depth+1))
        return None
    
    def ids(self,state):

        start_time = time.time()
        depth=0
        nodesExpanded = 0
        while(True):
            result = self.dls(state,depth,nodesExpanded)
            if result == None:
                depth = depth + 1
            else:
                result["runtime"] = time.time() - start_time
                return result
    
    def Astar(self,state,heuristic):

        start_time = time.time()
        if heuristic  == "manhattan":
            root = node(state,None,0,self.Manhattan_heuristic(state))
        else:
            root = node(state,None,0,self.Euclidean_heuristic(state))

        frontier = [root]
        explored = set()
        nodesExpanded = 0

        while len(frontier)!= 0:
            currentnode = heapq.heappop(frontier)
            explored.add(tuple(currentnode.state))
            nodesExpanded = nodesExpanded + 1

            if self.puzzle.is_goal(currentnode.state):
                runtime = time.time() - start_time
                path = currentnode.trace_path()
                return {
                        "path": path,
                        "cost": currentnode.cost,
                        "nodes_expanded": nodesExpanded,
                        "search_depth": len(path) - 1,
                        "runtime": runtime
                        }
            
            else:
                for neighbor in self.expand_node(currentnode,"A*",heuristic):
                    neighbor_state_tuple = tuple(neighbor.state)
                    if neighbor_state_tuple not in explored:
                        heapq.heappush(frontier,neighbor)
        return None

def main():
    
    # Create a puzzle instance
    puzzle_instance = puzzle([])
    initial_state = puzzle_instance.randomize_initial_state()
    print("Randomized Initial State:")
    print(initial_state)
    # Create a solver instance
    puzzle_solver = solver(puzzle_instance)

    # Test BFS
    print("Testing BFS:")
    bfs_solution = puzzle_solver.bfs(initial_state)
    
    puzzle_solver.display_path(bfs_solution)
    
    print("Solved in", len(bfs_solution) )

    #Test DFS
    print("\nTesting DFS:")
    dfs_solution = puzzle_solver.dfs(initial_state)
    print("DFS solution:", dfs_solution)
    print("solved in",len(dfs_solution))

    #Test IDS
    print("\nTesting IDS:")
    ids_solution = puzzle_solver.ids(initial_state)  # Set a reasonable max depth
    print("IDS solution:", ids_solution)
    print("solved in ", len(ids_solution))

    # Test A*
    print("\nTesting A* with Manhattan heuristic:")
    astar_solution_manhattan = puzzle_solver.Astar(initial_state, heuristic="manhattan")
    print("A* solution (Manhattan):", astar_solution_manhattan)
    print("solved in", len(astar_solution_manhattan))


    print("\nTesting A* with Euclidean heuristic:")
    astar_solution_euclidean = puzzle_solver.Astar(initial_state, heuristic="euclidean")
    print("A* solution (Euclidean):", astar_solution_euclidean)
    puzzle_solver.display_path(astar_solution_euclidean)
    print("solved in", len(astar_solution_euclidean))

if __name__ == "__main__":
    main()