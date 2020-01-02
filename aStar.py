import math
from heapq import heappush, heappop
from pyvisgraph.shortest_path import priority_dict
import queue as Q
from ass1 import print_maze, a_star


def euclidean_distance(start, end):
    return math.sqrt((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2)


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        if self.f == other.f:
            return self.h < other.h
        else:
            return self.f < other.f

    def __hash__(self):
        return hash(self.position)

    def get_neighbors(self, end_node, graph):
        children = []
        for edge in graph[self.position]:
            node_position = edge
            # Create new node
            new_node = Node(self, node_position)
            new_node.g = self.g + graph[self.position][node_position]['weight']
            new_node.h = euclidean_distance(node_position, end_node.position)
            new_node.f = new_node.g + new_node.h
            children.append(new_node)
        return children


def solution_path(current_node, maze, graph):
    path_cost = current_node.g
    path = []
    current = current_node
    while current is not None:
        path.append((current.position[0], current.position[1]))
        if maze is not None:
            maze[current.position[1]][current.position[0]] = 2
            if current.parent is not None:
                sub_path = graph[current.parent.position][current.position]['path'][1:-1][::-1]
                for point in sub_path:
                    path.append(point)
        current = current.parent
    if maze is not None:
        for point in path:
            maze[point[1]][point[0]] = 2
    print(path_cost)
    return path[::-1]  # Return reversed path
    # return path_cost


def aStar(maze, start, end, graph):
    # Create start and end node
    # start_node = Node(None, start)
    # start_node.h = euclidean_distance(start, end)
    # start_node.f = start_node.g + start_node.h
    # end_node = Node(None, end)

    # Initialize both open and closed list
    open_list_queue = Q.PriorityQueue()
    open_list = []
    closed_list = []

    # Add the start node
    # heappush(open_list, start_node)
    open_list_queue.put(start)
    open_list.append(start)
    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list_queue.get()
        if current_node not in open_list:
            continue
        open_list.remove(current_node)
        # Pop current off open list, add to closed list
        closed_list.append(current_node)
        # Found the goal
        if current_node.position == end.position:
            return solution_path(current_node, maze, graph)

        # Generate children
        children = current_node.get_neighbors(end, graph)

        # Loop through children
        for child in children:

            # Child is on the closed list
            if child in closed_list:
                continue

            # Child is already in the open list
            if child in open_list:
                dup_child = open_list[open_list.index(child)]
                if child.g < dup_child.g:

                    open_list.remove(dup_child)
                    open_list_queue.put(child)
            # Add the child to the open list
            else:
                open_list_queue.put(child)
                open_list.append(child)

def dijkstra(graph, origin, destination, maze):
    D = {}
    P = {}
    Q = priority_dict()
    Q[origin] = 0

    for v in Q:
        D[v] = Q[v]
        if v == destination: break

        edges = graph[v]
        for e in edges:
            w = e.get_adjacent(v)
            elength = D[v] + euclidean_distance(v, w)
            # aster_path_cost = a_star(maze, (int(v.x), int(v.y)), (int(w.x), int(w.y)), False)
            # elength = D[v] + aster_path_cost
            if w in D:
                if elength < D[w]:
                    raise ValueError
            elif w not in Q or elength < Q[w]:
                Q[w] = elength
                P[w] = v
    path = []
    while 1:
        path.append(destination)
        if destination == origin: break
        destination = P[destination]
    path.reverse()
    return path
