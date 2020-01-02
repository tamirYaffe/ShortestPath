import time
import matplotlib.pyplot as plt

import networkx as nx
from aStar import euclidean_distance, aStar


class Node:
    def __init__(self, parent=None, position=None, fuel=None):
        self.parent = parent
        self.position = position
        self.fuel = fuel
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return other and self.position == other.position and self.fuel == other.fuel

    def __lt__(self, other):
        if self.f == other.f:
            return self.h < other.h
        else:
            return self.f < other.f

    def __hash__(self):
        return hash((self.position, self.fuel))

    def get_neighbors(self, end_node, graph):
        children = []
        for edge in graph[self.position]:
            node_position = edge
            edge_weight = graph[self.position][node_position]['weight']
            if edge_weight > self.fuel:
                continue
            # Create new node
            new_node = Node(self, node_position, fuel=self.fuel - edge_weight + graph.nodes[node_position]['fuel'])
            new_node.g = self.g + edge_weight
            new_node.h = euclidean_distance(node_position, end_node.position)
            new_node.f = new_node.g + new_node.h
            children.append(new_node)
        return children


def add_edges(g, node):
    for other_node in g.nodes:
        if other_node == node:
            continue
        g.add_edge(node, other_node, weight=euclidean_distance(node, other_node))


def main():
    g = nx.Graph()
    g.add_node((0, 0), fuel=3)
    g.add_node((0, 3), fuel=3)
    g.add_node((3, 0), fuel=2)
    g.add_node((3, 3), fuel=5)
    g.add_node((6, 0), fuel=0)
    # add edges to g
    for node in g.nodes:
        add_edges(g, node)
    g.remove_edge((0, 0), (6, 0))
    origin = (0, 0)
    destination = (6, 0)
    start_node = Node(None, origin, 3)
    start_node.h = euclidean_distance(origin, destination)
    start_node.f = start_node.g + start_node.h
    end_node = Node(None, destination, 0)
    start_time = time.time()
    path = aStar(None, start_node, end_node, g)
    print("--- %s seconds ---" % (time.time() - start_time))
    prev_node = None
    solution_edges = []
    for node in path:
        if prev_node is not None:
            solution_edges.append((prev_node, node))
        prev_node = node
        print(node)

    # draw the graph
    pos = {}
    nodes_labels = {}
    for node in g.nodes:
        pos[node] = node
        nodes_labels[node] = g.nodes[node]['fuel']
    edge_labels = {}
    for edge in g.edges:
        edge_labels[edge] = round(g[edge[0]][edge[1]]['weight'], 2)
    nx.draw_networkx_nodes(g, pos, nodelist=[start_node.position], node_size=500, node_color='g')
    nx.draw_networkx_nodes(g, pos,
                           nodelist=[node for node in g.nodes if node not in [start_node.position, end_node.position]],
                           node_size=500)
    nx.draw_networkx_nodes(g, pos, nodelist=[end_node.position], node_size=500, node_color='r')
    nx.draw_networkx_edges(g, pos, edgelist=solution_edges, alpha=1, width=4, edge_color='r')
    nx.draw_networkx_edges(g, pos, edgelist=[edge for edge in g.edges if edge not in solution_edges], alpha=1, width=4)
    nx.draw_networkx_labels(g, pos, nodes_labels, font_size=10)
    nx.draw_networkx_edge_labels(g, pos, edge_labels, label_pos=0.4, font_size=8)
    plt.axis('on')
    plt.show()


if __name__ == '__main__':
    main()
