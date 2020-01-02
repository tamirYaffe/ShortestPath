import math
import time
from heapq import heappush, heappop



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
        if self.f != other.f:
            return self.f < other.f
        else:
            return self.g < other.g



    def __hash__(self):
        return hash(self.position)

    def get_neighbors(self, maze, end_node):
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:  # Adjacent squares

            # Get node position
            node_position = (self.position[0] + new_position[0], self.position[1] + new_position[1])

            # Make sure within range
            if node_position[1] > (len(maze) - 1) or node_position[1] < 0 or\
                    node_position[0] > (len(maze[len(maze) - 1]) - 1) or node_position[0] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[1]][node_position[0]] != 0:
                continue

            # Create new node
            new_node = Node(self, node_position)
            if new_position in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
                new_node.g = self.g + math.sqrt(2)
            else:
                new_node.g = self.g + 1
            euclidean_distance = math.sqrt((new_node.position[0] - end_node.position[0]) ** 2 + (new_node.position[1] - end_node.position[1]) ** 2)
            new_node.h = euclidean_distance
            new_node.f = new_node.g + new_node.h

            # Append
            children.append(new_node)
        return children



def a_star(maze, start, end, return_path):
    # Create start and end node
    start_node = Node(None, start)
    end_node = Node(None, end)

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    heappush(open_list, start_node)
    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = heappop(open_list)
        # Pop current off open list, add to closed list
        closed_list.append(current_node)

        # Found the goal
        # if current_node == end_node:
        if current_node == end_node:
            path_cost = current_node.g
            if return_path is False:
                return path_cost
            path = []
            current = current_node
            while current is not None:
                # maze[current.position[1]][current.position[0]] = 2
                path.append(current.position)
                current = current.parent
            # print_maze(maze)
            return path[::-1], path_cost # Return reversed path

        # Generate children
        children = current_node.get_neighbors(maze, end_node)

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
                    heappush(open_list, child)
                # for open_node in open_list:
                #     if child == open_node and child.g > open_node.g:
                #         continue
            # Add the child to the open list
            else:
                heappush(open_list, child)


def euclidean_distance(start, end):
    return math.sqrt((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2)

def main():
    map_file = open("maps/arena2.map", "r")
    maze = make_maze_from_file(map_file)
    map_file.close()
    origin = (91, 2)
    destination = (219, 191)
    start_time = time.time()
    path = a_star(maze, origin, destination, True)
    print(path)
    print("--- %s seconds ---" % (time.time() - start_time))

def make_maze_from_file(map_file):
    lines = []
    for line in map_file:
        lines.append(line)
    height = int(lines[1].split()[1])
    width = int(lines[2].split()[1])
    maze = [[0 for x in range(width)] for y in range(height)]
    for i in range(0, len(maze)):
        maze_row = lines[i+4]
        for j in range(0, len(maze[0])):
            cell = 1
            if maze_row[j] == ".":
                cell = 0
            maze[i][j] = cell
    # print_maze(maze)
    return maze


def print_maze(maze):
    for r in maze:
        for c in r:
            print(c, end=" ")
        print()

def a_star_v2(maze, start, end, return_path):
    # Create start and end node
    start_node = Node(None, start)
    end_node = Node(None, end)

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    heappush(open_list, start_node)
    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = heappop(open_list)
        # Pop current off open list, add to closed list
        closed_list.append(current_node)

        # Found the goal
        # if current_node == end_node:
        if current_node == end_node or euclidean_distance(current_node.position, end_node.position) < 2:
            path_cost = current_node.g
            path = []
            current = current_node
            while current is not None:
                # maze[current.position[1]][current.position[0]] = 2
                path.append(current.position)
                current = current.parent
            # print_maze(maze)
            if return_path:
                return path[::-1] # Return reversed path
            else:
                return path_cost

        # Generate children
        children = current_node.get_neighbors(maze, end_node)

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
                    heappush(open_list, child)
                # for open_node in open_list:
                #     if child == open_node and child.g > open_node.g:
                #         continue
            # Add the child to the open list
            else:
                heappush(open_list, child)

if __name__ == '__main__':
    main()

