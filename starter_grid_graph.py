'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   Name:          grid_graph_starter.py
   By:            Qin Chen
   Last Updated:  6/10/18
    
   Definition of class GridGraph. Description of all the methods is
   provided. Students are expected to implement the methods for Lab#6.
   ========================================================================*/
'''
import Tkinter as tk
import starter_grid_graph_display as display
from bfsDrive import *

class GridGraph(object):
    def __init__(self):
        self.nodes = {} # {node_name: set(neighboring nodes), ...}
        self.startNode = None  # string
        self.goalNode = None    # string
        self.grid_rows = None
        self.grid_columns = None
        self.obs_list = []
        self.node_display_locations=[]
        return

    # set number of rows in the grid
    def set_grid_rows(self, rows):
       self.grid_rows=rows
       return

    # set number of columns in the grid
    def set_grid_cols(self, cols):
        self.grid_columns=cols
        return

    # this method is used by make_grid() to create a key-value pair in self.nodes{},
    # where value is created as an empty set which is populated later while connecting
    # nodes.
    def add_node(self, name):
        self.nodes[name] = set([])

    # set start node name
    def set_start(self, name):
        self.startNode = name

    # returns start node name
    def get_start_node(self):
        return self.startNode

    # set goal node name
    def set_goal(self, name):
        self.goalNode=name

    # return goal node name
    def get_goal_node(self):
        return self.goalNode

    # Given two neighboring nodes. Put them to each other's neighbors-set. This
    # method is called by self.connect_nodes() 
    def add_neighbor(self, node1, node2):
        self.nodes[node1].add(node2)
        self.nodes[node2].add(node1)
        return

    # populate graph with all the nodes in the graph, excluding obstacle nodes
    def make_grid(self):
        for column in range (0, self.grid_columns):
            for row in range (0,self.grid_rows):
                if [column,row] not in self.obs_list:
                    self.add_node((str(column) + "-" + str(row)))
    # Based on node's name, this method identifies its neighbors and fills the 
    # set holding neighbors for every node in the graph.

    def connect_nodes(self):
        for row in range(0,self.grid_rows):
            for col in range(0, self.grid_columns):
                if ([col,row] not in self.obs_list and self.nodes.has_key(str(col) + '-' + str(row))):
                    if [col+1, row] not in self.obs_list and self.nodes.has_key(str(col+1) + '-' + str(row)):
                         self.add_neighbor(str(col) + "-" + str(row), (str(col+1) + "-" + str(row)))
                    if [col, row+1] not in self.obs_list and self.nodes.has_key(str(col) + '-' + str(row+1)):
                         self.add_neighbor(str(col) + "-" + str(row), (str(col) + "-" + str(row+1)))

    # For display purpose, this function computes grid node location(i.e., offset from upper left corner where is (1,1)) 
    # of display area. based on node names.
    # Node '0-0' is displayed at bottom left corner 
    def compute_node_locations(self):
        #scaleFactor=1
        for column in range (0,self.grid_columns):
            for row in range (0,self.grid_rows):
                if [column,row] not in self.obs_list:
                    self.node_display_locations.append([column,row])
        return


###########################################################
#  A testing program of your implementaion of GridGraph class.
###########################################################
def main():
    # instantiate COMM object
    gMaxRobotNum = 1; # max number of robots to control
    comm = RobotComm(gMaxRobotNum)
    comm.start()
    print 'Bluetooth starts'
    robotList = comm.robotList

    graph = GridGraph()
    # grid dimension
    graph.set_grid_rows(4)
    graph.set_grid_cols(3)

    # origin of grid is (0, 0) lower left corner
    # graph.obs_list = ([1,1],)    # in case of one obs. COMMA
    graph.obs_list = ([1,1], [0, 3], [2,2])
    
    graph.set_start('0-0')
    graph.set_goal('2-3')
    
    graph.make_grid()
    graph.connect_nodes()
    graph.compute_node_locations()

    frame = tk.Tk()
    gui = display.GridGraphDisplay(frame, graph)
    gui.display_graph()

    behaviors = RobotBehaviorThread(robotList, gui.DriveCommands, gui)
    behaviors.start()

    frame.mainloop()

    return

if __name__ == "__main__":
    main()