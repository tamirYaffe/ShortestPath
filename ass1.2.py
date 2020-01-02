import math
import time
from bresenham import bresenham
import numpy as np
import rasterio.features
import networkx as nx

from GUI import CellGrid
from aStar import aStar, euclidean_distance, Node
from ass1 import make_maze_from_file, a_star, a_star_v2
from tkinter import *


def add_edges_to_g(g, maze, node):
    for node2 in g.nodes:
        if node == node2:
            continue
        has_los = True
        los_path = list(bresenham(node[0], node[1], node2[0], node2[1]))
        los_path_cost = 0
        prev_point = los_path[0]
        for point in los_path[1:-1]:
            if maze[point[1]][point[0]] == 1:
                has_los = False
                break
            dif = (prev_point[0]-point[0], prev_point[1]-point[1])
            if dif in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
                los_path_cost = los_path_cost + math.sqrt(2)
            else:
                los_path_cost = los_path_cost + 1
            prev_point = point
        if has_los is False:
            continue
        # Create new edge
        point = los_path[-1]
        dif = (prev_point[0] - point[0], prev_point[1] - point[1])
        if dif in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
            los_path_cost = los_path_cost + math.sqrt(2)
        else:
            los_path_cost = los_path_cost + 1
        g.add_edge(node, node2, weight=los_path_cost, path=los_path)


def buildVisGraph(maze, load):
    if load:
        return nx.read_gpickle("graph.gpickle")
    # Build a numpy array
    myarray = np.array(maze)
    # Convert the type (don't even know why this was needed in my computer, but raised exception if not converted.
    myarray = myarray.astype(np.int32)

    # Let the library do the magic. You should take a look at the rasterio.features.shapes output
    mypols = [p[0]['coordinates']
              for p in rasterio.features.shapes(myarray)]
    # transform mypols to polys of vg.Point
    polys = []
    frame_pol = []
    frame_top_left = (0, 0)
    frame_top_right = (len(maze[0]), 0)
    frame_bottom_left = (0, len(maze))
    frame_bottom_right = (len(maze[0]), len(maze))
    frame_pol.append(frame_top_left)
    frame_pol.append(frame_top_right)
    frame_pol.append(frame_bottom_left)
    frame_pol.append(frame_bottom_right)
    for mypol in mypols:
        pol = []
        for point in mypol[0]:
            if point not in pol:
                pol.append(point)
        #  check frame pol excluded
        excluded = False
        for point in frame_pol:
            if point not in pol:
                excluded = True
        if excluded:
            polys.append(pol)
    # delete container polygon
    # del polys[-1]
    g = nx.Graph()
    # add nodes to g
    for poly in polys:
        for point in poly:
            for new_position in [(0, -1), (0, 0), (-1, 0), (-1, -1)]:  # Adjacent squares
                # Get node position
                new_point = (int(point[0] + new_position[0]), int(point[1] + new_position[1]))

                # Make sure within range
                if new_point[1] > (len(maze) - 1) or new_point[1] < 0 or \
                        new_point[0] > (len(maze[len(maze) - 1]) - 1) or new_point[0] < 0:
                    continue

                # Make sure walkable terrain
                if maze[int(new_point[1])][int(new_point[0])] != 0:
                    continue

                g.add_node(new_point)
    # add edges to g
    for node in g.nodes:
        add_edges_to_g(g, maze, node)

    # print("start building vis graph...")
    # g = vg.VisGraph()
    # g.build(polys, 4, True)
    nx.write_gpickle(g, "graph.gpickle")
    return g


def add_missing_edges(g, origin, destination, maze):
    origin_exists = origin in g.nodes
    dest_exists = destination in g.nodes
    if origin_exists and dest_exists:
        return
    if dest_exists is False:
        g.add_node(destination)

    if not origin_exists:
        g.add_node(origin)
        add_edges_to_g(g, maze, origin)
    if not dest_exists:
        add_edges_to_g(g, maze, destination)


def main():
    maze = [[1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 0, 1, 1, 0, 0, 1],
            [1, 0, 0, 0, 1, 1, 0, 0, 1],
            [1, 0, 0, 0, 1, 1, 0, 0, 1],
            [1, 0, 0, 0, 1, 1, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1]]

    map_file = open("maps/arena2.map", "r")
    maze = make_maze_from_file(map_file)
    map_file.close()
    g = buildVisGraph(maze, True)
    # origin = (1, 1)
    # destination = (2, 7)
    # origin = vg.Point(45.0, 5.0)
    # destination = vg.Point(6.0, 28.0)
    origin = (91, 2)
    destination = (219, 191)
    # origin = (40, 1)
    # destination = (8, 79)
    add_missing_edges(g, origin, destination, maze)
    start_node = Node(None, origin)
    start_node.h = euclidean_distance(origin, destination)
    start_node.f = start_node.g + start_node.h
    end_node = Node(None, destination)
    print("starting astar")
    start_time = time.time()
    path = aStar(maze, start_node, end_node, g)
    # path = dijkstra(g.visgraph, origin, destination, maze)
    # path = g.shortest_path(origin, destination)
    # print(path)
    print("--- %s seconds ---" % (time.time() - start_time))
    for point in path:
        maze[point[1]][point[0]] = 6
    maze[origin[1]][origin[0]] = 2
    maze[destination[1]][destination[0]] = 3
    root = Tk()
    my_gui = CellGrid(root, len(maze), len(maze[0]), 5, maze)
    root.mainloop()


def test():
    maze = [[1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 0, 1, 1, 0, 0, 1],
            [1, 0, 0, 0, 1, 1, 0, 0, 1],
            [1, 0, 0, 0, 1, 1, 0, 0, 1],
            [1, 0, 0, 0, 1, 1, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1]]

    for point in list(bresenham(1, 1, 3, 2)):
        print(point)


if __name__ == '__main__':
    main()
    # test()