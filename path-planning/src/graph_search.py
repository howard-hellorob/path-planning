import numpy as np
from .graph import Cell
from .utils import trace_path
from collections import deque
import heapq

"""
General graph search instructions:

First, define the correct data type to keep track of your visited cells
and add the start cell to it. If you need to initialize any properties
of the start cell, do that too.

Next, implement the graph search function. When you find a path, use the
trace_path() function to return a path given the goal cell and the graph. You
must have kept track of the parent of each node correctly and have implemented
the graph.get_parent() function for this to work. If you do not find a path,
return an empty list.

To visualize which cells are visited in the navigation webapp, save each
visited cell in the list in the graph class as follows:
     graph.visited_cells.append(Cell(cell_i, cell_j))
where cell_i and cell_j are the cell indices of the visited cell you want to
visualize.
"""


def depth_first_search(graph, start, goal):
    """Depth First Search (DFS) algorithm. This algorithm is optional for P3.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    """TODO (P3): Implement DFS (optional)."""
    if graph.check_cell_collision(start.i, start.j):
        print("Start cell in collision")
        return []
    if graph.check_collision(goal.i, goal.j):
        return []
    
    stack = [start]

    graph.nodes[start.j, start.i].visited = True
    graph.nodes[start.j, start.i].distance = 0

    while stack:
        current = stack.pop()

        graph.visited_celss.append(Cell(current.i, current.j))

        if current.i == goal.i and current.j == goal.j:
            return trace_path(goal, graph)
        
        neighbors = graph.find_neighbors(current.i, current.j)
        for neighbor in neighbors:
            if graph.nodes[neighbor.j, neighbor.i].visited:
                continue
            if graph.check_collision(neighbor.i, neighbor.j):
                continue

            graph.nodes[neighbor.j, neighbor.i].visited = True
            graph.nodes[neighbor.j, neighbor.i].parent = Cell(current.i, current.j)
            graph.nodes[neighbor.j, neighbor.i].distance = graph.nodes[current.j, current.i].distance + 1

            stack.append(neighbor)
        

    # If no path was found, return an empty list.
    return []


def breadth_first_search(graph, start, goal):
    """Breadth First Search (BFS) algorithm.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    """TODO (P3): Implement BFS."""
    if graph.check_collision(start.i, start.j):
        print("Start cell in collision")
        return []
    if graph.check_collision(goal.i, goal.j):
        return []
    
    queue = deque([start])

    graph.nodes[start.j, start.i].visited = True
    graph.nodes[start.j, start.i].distance = 0

    while queue:
        current = queue.popleft()

        graph.visited_cells.append(Cell(current.i, current.j))

        if current.i == goal.i and current.j == goal.j:
            return trace_path(goal, graph)
        
        neighbors = graph.find_neighbors(current.i, current.j)
        for neighbor in neighbors:
            if graph.nodes[neighbor.j, neighbor.i].visited:
                continue
            if graph.check_collision(neighbor.i, neighbor.j):
                continue

            graph.nodes[neighbor.j, neighbor.i].visited = True
            graph.nodes[neighbor.j, neighbor.i].parent = Cell(current.i, current.j)
            graph.nodes[neighbor.j, neighbor.i].distance = graph.nodes[current.j, current.i].distance + 1

            queue.append(neighbor)


    # If no path was found, return an empty list.
    return []

def heuristic(cell1, cell2):
    return abs(cell1.i - cell2.i) + abs(cell1.j - cell2.j)


def a_star_search(graph, start, goal):
    """A* Search (BFS) algorithm.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    """TODO (P3): Implement A*."""
    if graph.check_collision(start.i, start.j):
        print("Start cell in collision")
        return []
    if graph.check_collision(goal.i, goal.j):
        return []
    
    counter = 0
    pq = [(0, counter, start)]

    graph.nodes[start.j, start.i].g_cost = 0
    graph.nodes[start.j, start.i].h_cost = heuristic(start, goal)
    graph.nodes[start.j, start.i].cost = graph.nodes[start.j, start.i].g_cost + graph.nodes[start.j, start.i].h_cost
    graph.nodes[start.j, start.i].visited = True

    while pq:
        f_cost, _, current = heapq.heappop(pq)

        graph.visited_cells.append(Cell(current.i, current.j))

        if current.i == goal.i and current.j == goal.j:
            return trace_path(goal, graph)
        
        current_g = graph.nodes[current.j, current.i].g_cost

        neighbors = graph.find_neighbors(current.i, current.j)
        for neighbor in neighbors:
            if graph.check_collision(neighbor.i, neighbor.j):
                continue

            tentative_g = current_g + 1

            if tentative_g < graph.nodes[neighbor.j, neighbor.i].g_cost:
                graph.nodes[neighbor.j, neighbor.i].parent = Cell(current.i, current.j)
                graph.nodes[neighbor.j, neighbor.i].g_cost = tentative_g
                graph.nodes[neighbor.j, neighbor.i].h_cost = heuristic(neighbor, goal)
                graph.nodes[neighbor.j, neighbor.i].cost = graph.nodes[neighbor.j, neighbor.i].g_cost + graph.nodes[neighbor.j, neighbor.i].h_cost

                if not graph.nodes[neighbor.j, neighbor.i].visited:
                    counter += 1
                    heapq.heappush(pq, (graph.nodes[neighbor.j, neighbor.i].cost, counter, neighbor))
                    graph.nodes[neighbor.j, neighbor.i].visited = True

    # If no path was found, return an empty list.
    return []
