
# coding: utf-8

# In[8]:

class Node:
    """ Implementation of the node structure in our graph. """


    """
    Creates a new node.

    Args:
      node_id: Integer.
      node_type: Integer. One of 1, 2, 3, or 4, corresponding to 'sidewalk',
          'road', 'entrance', and 'exit', respectively.
      x: Integer. The x-coordinate of the node in the grid.
      y: Integer. The y-coordinate of the node in the grid.

    Returns:
      A new Node object.

    """
    def __init__(self, node_id, node_type, x, y):
        # A unique identifier for the node.
        self.node_id = node_id

        # The node type.
        self.node_type = node_type

        # X and y coordinates for the node in the grid.
        self.x = x
        self.y = y

        # Initialize a dictionary to hold the next node in the shortest path
        # for each destination.
        self.paths = {}

        # Array of Node neighbors.
        self.neighbors = {}

        # By default, the node is not occupied (i.e., it is available).
        self.available = True

    # For a given destination node, returns the next node in the shortest path.
    def get_next_node(self, dest, node_dict):
        next_node_id = self.paths[dest.node_id]

        return node_dict[next_node_id]

class Edge:
    """ Implementation of the edge structure in our graph. """


    """
    Creates a new edge.

    Args:
      node_a: Node.
      node_b: Node.
      weight: Float. The weight or cost of the edge.

    Returns:
      A new Edge object.

    """
    def __init__(self, node_a, node_b, weight):
        self.node_a = node_a
        self.node_b = node_b
        self.weight = weight


# In[9]:

class Intersection:
    """
    Implements an intersection, used for parametrically limiting pedestrian flow.
    Note: the intersection is not yet used in this initial model, but its definition
        is provided here.
    """


    """
    Creates a new intersection.

    Args:
      int_id: Integer. Unique numerical identifier for the intersection.
      nodes: List. List of Node objects that are part of the intersection.

    Returns:
      A new Intersection object.

    """
    def __init__(self, int_id, nodes=[]):
        # Unique identifier.
        self.int_id = int_id
        # List of nodes that are in the intersection
        self.nodes = nodes
        # State of whether the intersection is available to pedestrians (default - closed to peds).
        self.is_open = False

    # Close all the nodes in the intersection.
    def close_me():
        # Need to handle case when pedestrian is in the middle of an intersection during closing.
        for node in self.nodes:
            node.available = False

    # Re-open the intersection by setting all included nodes to available.
    def open_me():
        for node in self.nodes:
            node.available = True


# In[10]:

class Pedestrian:
    """ Implements a pedestrian. """


    """
    Creates a new pedestrian.

    Args:
      current: Node. A Node object corresponding to the current location of the
        pedestrian.
      destination: Node. A Node object corresponding to the destination node.
      speed: Integer. Number of grid cells traversed per time step.
      node_dict: Dictionary. Lookup table of node_id -> node object for every node
        in the simulation.

    Returns:
      A new Pedestrian object.

    """
    def __init__(self, current, destination, speed, node_dict):
        # The current location of the pedestrian, as a Node.
        self.current = current

        # The pedestrian's final destination, as a Node.
        self.destination = destination

        # The speed of the pedestrian, formulated as an integer number of
        # grid cells traversed per time step.
        self.speed = speed

        # The desired next node to move to.
        self.target_next = current.get_next_node(self.destination, node_dict)

        # Whether the pedestrian has completed egress (i.e., exited the SUI).
        self.egress_complete = False

    # Move the pedestrian to a given node. Takes a node (Node), node_dict (Dictionary),
    # and type_map (Dictionary) translating string node types to node_type ids.
    def move(self, node, node_dict, type_map):
        # If the requested node is not available to move to, return.
        if not node.available:
            return

        # The current node is now available.
        self.current.available = True

        # Update the current node to the new node.
        self.current = node

        # If current is a destination node,
        if self.current.node_type == type_map['exit']:
            # Egress is completed.
            self.egress_complete = True
        else:
            # Set the new current node to unavailable.
            self.current.available = False

            # Update the target next.
            self.target_next = node.get_next_node(self.destination, node_dict)

        return self


# In[11]:

# Dijkstra's algorithm for shortest paths.
# Implementation adapted from online version published by David Eppstein, UC Irvine, 4 April 2002.
# Accessible at https://www.ics.uci.edu/~eppstein/161/python/dijkstra.py.

from priodict import priorityDictionary

class ShortestPath:
    """ An abstraction for finding the shortest path between two nodes in a graph. """


    """
    Creates a new ShortestPath instance.

    Args:
      graph: Dictionary. Should be a dictionary with keys corresponding to each node_id
        in the graph. Each key should itself point to another dictionary, with keys corresponding
        to neighboring nodes, and values corresponding to edge weights.
      start: Integer. The node_id of the starting node.
      end: Integer: The node_id of the destination node.

    Returns:
      A new ShortestPath object.

    """
    def __init__(self, graph, start, end):
        self.graph = graph
        self.start = start
        self.end = end

    # Returns the next node in the shortest path.
    def next_node(self):
        try:
            shortest_path = self.shortest_path(self.graph, self.start, self.end)
        except Exception:
            return None

        return shortest_path[1]

    # Implements Dijkstra's algorithm, given a graph G, start node_id, and end node_id.
    def dijkstra(self, G, start, end=None):
        """
        Find shortest paths from the  start vertex to all vertices nearer than or equal to the end.

        The input graph G is assumed to have the following representation:
        A vertex can be any object that can be used as an index into a dictionary.
        G is a dictionary, indexed by vertices.  For any vertex v, G[v] is itself a dictionary,
        indexed by the neighbors of v.  For any edge v->w, G[v][w] is the length of the edge.
        This is related to the representation in <http://www.python.org/doc/essays/graphs.html>
        where Guido van Rossum suggests representing graphs as dictionaries mapping vertices
        to lists of outgoing edges, however dictionaries of edges have many advantages over lists:
        they can store extra information (here, the lengths), they support fast existence tests,
        and they allow easy modification of the graph structure by edge insertion and removal.
        Such modifications are not needed here but are important in many other graph algorithms.
        Since dictionaries obey iterator protocol, a graph represented as described here could
        be handed without modification to an algorithm expecting Guido's graph representation.

        Of course, G and G[v] need not be actual Python dict objects, they can be any other
        type of object that obeys dict protocol, for instance one could use a wrapper in which vertices
        are URLs of web pages and a call to G[v] loads the web page and finds its outgoing links.

        The output is a pair (D,P) where D[v] is the distance from start to v and P[v] is the
        predecessor of v along the shortest path from s to v.

        Dijkstra's algorithm is only guaranteed to work correctly when all edge lengths are positive.
        This code does not verify this property for all edges (only the edges examined until the end
        vertex is reached), but will correctly compute shortest paths even for some graphs with negative
        edges, and will raise an exception if it discovers that a negative edge has caused it to make a mistake.
        """

        D = {}	# dictionary of final distances
        P = {}	# dictionary of predecessors
        Q = priorityDictionary()	# estimated distances of non-final vertices
        Q[start] = 0

        for v in Q:
            D[v] = Q[v]
            if v == end: break

            for w in G[v]:
                vwLength = D[v] + G[v][w]
                if w in D:
                    if vwLength < D[w]:
                        raise ValueError, "Dijkstra: found better path to already-final vertex"
                elif w not in Q or vwLength < Q[w]:
                    Q[w] = vwLength
                    P[w] = v

        return (D,P)

    # Finds the shortest path given a graph G, start node_id, and end node_id.
    def shortest_path(self, G, start, end):
        """
        Find a single shortest path from the given start vertex to the given end vertex.
        The input has the same conventions as Dijkstra().
        The output is a list of the vertices in order along the shortest path.
        """

        D,P = self.dijkstra(G,start,end)
        Path = []
        while 1:
            Path.append(end)
            if end == start: break
            end = P[end]
        Path.reverse()
        return Path


# In[24]:

import csv

class Reader(object):
    """
    Generic Reader class for processing input. Should be subclassed instead of used directly.
    """


    def __init__(self, filename):
        self.filename = filename

        return self.process()

    def process(self):
        # Override in subclass.
        pass

class NodeReader(Reader):
    """
    Reads in vertices from a CSV input file, creating new Node objects for each.
    Saves those node objects to a node_dict Dictionary indexed by node_id.
    """


    def __init__(self, filename):
        # Initialize a node container and a node dictionary.
        self.nodes = []
        self.node_dict = {}

        # Call __init__ on the parent class.
        super(NodeReader, self).__init__(filename)

    def process(self):
        with open(self.filename, 'rb') as csvfile:
            # Skip the first line.
            next(csvfile)

            # Create a CSV reader.
            reader = csv.reader(csvfile, delimiter=',')

            for indx, row in enumerate(reader):
                # The node_id for the Node will be indx.
                node_id = indx

                # Node type.
                node_type = int(row[6])

                # X and y coordinates.
                x = int(row[0])
                y = int(row[1])

                # Create a new node.
                newnode = Node(node_id, node_type, x, y)

                # Append it to the nodes array.
                self.nodes.append(newnode)

                # Add an entry in the node dictionary.
                self.node_dict[node_id] = newnode

        return self

class EdgeReader(Reader):
    """
    Reads in edges from a CSV input file, creating new Edge objects for each.
    Saves those edge objects to an edges list.
    """


    def __init__(self, filename):
        # Initialize an edges container.
        self.edges = []

        # Call __init__ on the parent class.
        super(EdgeReader, self).__init__(filename)

    def process(self):
        with open(self.filename, 'rb') as csvfile:
            # Skip the first line.
            next(csvfile)

            # Create a CSV reader.
            reader = csv.reader(csvfile, delimiter=',')

            for row in reader:
                # The node_a and node_b corresponding to node_ids.
                node_a = int(row[0])
                node_b = int(row[1])

                # The weight is a float value that will be used for computations.
                weight = float(row[2])

                # Create a new edge.
                newedge = Edge(node_a, node_b, weight)

                # Append it to the edges array.
                self.edges.append(newedge)

        return self

class Printer:
    """
    Simple printer class that provides static methods for printing text to STDOUT.
    """


    def __init__(self):
        pass

    @staticmethod
    def pp(text, char='*', length=75):
        print(char*length)
        print(text)
        print(char*length)

import pickle

class Grid:
    """
    Builds a grid used for translating the graph into a meaningful 2D arrangement.
    """


    def __init__(self, max_rows, max_cols, node_file, edge_file, type_map, paths_file=None):
        # Save some important attributes.
        self.max_rows = max_rows
        self.max_cols = max_cols
        self.node_file = node_file
        self.edge_file = edge_file
        self.type_map = type_map
        self.paths_file = None

        # Perform initialization of the gridspace.
        self.initialize_grid()
        self.initialize_nodes()
        self.initialize_edges()
        self.set_paths()

    def initialize_grid(self):
        # Create a grid with the given number of rows, cols.
        self.grid = [[0 for x in range(self.max_cols)] for x in range(self.max_rows)]

    def initialize_nodes(self):
        reader = NodeReader(self.node_file)

        # Save the node dict for later lookups.
        self.node_dict = reader.node_dict

        # Save off the node array.
        self.nodes = reader.nodes

        # Initialize a list container of entrance nodes.
        self.entrance_nodes = []

        # Initialize a list container of destination nodes.
        self.destination_nodes = []

        # Iterate through the returned nodes,
        for node in self.nodes:
            # If the node is an entrance node, add it to the entrance nodes list.
            if node.node_type == self.type_map['entrance']:
                self.entrance_nodes.append(node)
            # If the node is an exit node, add it to the exit nodes list.
            elif node.node_type == self.type_map['exit']:
                self.destination_nodes.append(node)

            # Ad each node to the grid in the appropriate location.
            self.grid[node.y-1][node.x-1] = node

    def initialize_edges(self):
        reader = EdgeReader(self.edge_file)

        # Save off the edges array.
        edges = reader.edges

        for edge in edges:
            # Look up the first node.
            node_a = self.node_dict[edge.node_a]

            # Look up the second node to make sure it exists.
            node_b = self.node_dict[edge.node_b]

            # Add a new entry to node a's neighbors dict for node b, setting it
            # to the weight.
            node_a.neighbors[node_b.node_id] = edge.weight

            # Added to make undirected.
            node_b.neighbors[node_a.node_id] = edge.weight

        # Initialize a dictionary to store just the neighbors.
        self.neighbors_dict = {}

        # For every entry in the node dictionary,
        for node_id, node_obj in self.node_dict.iteritems():
            # Save just the neighbors.
            self.neighbors_dict[node_id] = node_obj.neighbors

    def set_paths(self):
        num_nodes = len(self.nodes)

        Printer.pp('Performing preprocessing step to find shortest paths. Please bear with us.')

        for indx, node in enumerate(self.nodes):
            if node.node_type == 4:
                continue

            node_id = node.node_id

            for destination in self.destination_nodes:
                destination_node_id = destination.node_id

                node.paths[destination_node_id] = ShortestPath(self.neighbors_dict,
                                                               node_id,
                                                               destination_node_id).next_node()

            if indx % 100 == 0 and indx != 0:
                percent_done = ((indx+1)/float(num_nodes))*100
                print('%d percent done.' % percent_done)

        print('---> Preprocessing done.')

# Note: we will be developing a custom random number generator for the final
# simulation.
import random
import matplotlib
import matplotlib.pyplot as plt
import time

class Simulation:
    """
    Performs a cellular automata simulation using an input Grid object.
    """


    def __init__(self, grid, params = {}):
        self.grid = grid
        self.num_pedestrians = params.get('num_pedestrians', 500)
        self.visualization = params.get('visualization', False)

        self.seed_pedestrians()

    def seed_pedestrians(self):
        num_entrance_nodes = len(self.grid.entrance_nodes)
        num_destination_nodes = len(self.grid.destination_nodes)

        self.ped_queue = [self.generate_pedestrian(num_entrance_nodes, num_destination_nodes)
                          for i in range(self.num_pedestrians)]

    def generate_pedestrian(self, num_entrance_nodes, num_destination_nodes):
        # TODO: add random number generator.
        entrance_node = self.grid.entrance_nodes[random.randrange(0, num_entrance_nodes-1)]
        destination_node = self.grid.destination_nodes[random.randrange(0, num_destination_nodes-1)]

        # TODO: set speed (third argument) dynamically by drawing from a distribution.
        new_ped = Pedestrian(entrance_node, destination_node, 1, self.grid.node_dict)

        return new_ped

    # Initialize the Visualization plot.
    def init_viz(self):
        # Set interactive plot on and create figure.
        plt.ion()
        fig = plt.figure()
        self.ax = fig.add_subplot(1, 1, 1)
        self.scat = self.ax.scatter([], [], zorder=1)

        # Set background image.
        img = plt.imread('playMat.png')
        plt.imshow(img, extent=(0, 140, 70, 0), zorder=0)
        plt.show()

    def update_viz(self, x_vals, y_vals):
        # Clear existing points.
        self.scat.remove()

        # Update points to be plotted.
        self.scat = self.ax.scatter(x_vals, y_vals, zorder=1)
        # plt.scatter(random.randrange(0, 20), random.randrange(0, 15))

        # Draw.
        plt.draw()

        # A short pause so Mac OS X 10.11.3 doesn't break.
        plt.pause(0.0001)

    def run(self):
        Printer.pp('Initializing simulation.')

        self.run_simulation()

    def run_simulation(self):
        active_peds = []

        if self.visualization:
            self.init_viz()

        while True:
            # Initialize for viz.
            if self.visualization:
                x_vals = []
                y_vals = []

            target_next_dict = {node: [] for node in self.grid.nodes}

            if len(self.ped_queue) == 0 and len(active_peds) == 0:
                print('Finished!')
                break

            if len(self.ped_queue) > 0:
                next_ped = self.ped_queue[0]

                if next_ped.current.available:
                    active_peds.append(self.ped_queue.pop(0))

            active_peds_remaining = len(active_peds)

            if active_peds_remaining % 10 == 0:
                print('%d active peds remaining to evacuate.' % active_peds_remaining)

            for indx, ped in enumerate(active_peds):
                # If the ped is finished,
                if ped.egress_complete:
                    # Remove her.
                    del active_peds[indx]
                    continue

                # Grab the ped's target next node according to the shortest path algo.
                target_next = ped.target_next

                # Add an entry in the node => peds dictionary, telling us that the ped
                # wants to go to that node.
                target_next_dict[target_next].append(ped)

                # Get x,y values for viz
                if self.visualization:
                    x_vals.append(ped.current.x)
                    y_vals.append(ped.current.y)

            for node, peds in target_next_dict.iteritems():
                ped_len = len(peds)

                if ped_len == 0:
                    # No pedestrian wants the node, so go to the next iteration.
                    continue
                elif ped_len == 1:
                    # We're done; only one ped wants the node.
                    ped = peds[0]
                else:
                    # Select a ped at random to get the node.
                    ped = peds[random.randrange(0, ped_len-1)]

                # Move the ped.
                ped.move(node, self.grid.node_dict, self.grid.type_map)

            # Update viz.
            if self.visualization:
                self.update_viz(x_vals, y_vals)

        print('Simulation completed.')

        #time.sleep(3)
        #plt.close()

max_rows = 66
max_cols = 139

type_map = { 'sidewalk': 1, 'crosswalk': 2, 'entrance': 3, 'exit': 4 }
grid = Grid(max_rows, max_cols, 'playMat.png.vertex.stripped', 'playMat.png.edge.stripped', type_map)

import timeit

simulation = Simulation(grid, {'num_pedestrians': 500, 'visualization': True})
print(timeit.Timer(simulation.run).timeit(number=1))