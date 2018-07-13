# Robot Programming
# breadth first search
# by Dr. Qin Chen
# May, 2016

import sys
import Tkinter as tk
from starter_bfs import *

##############
# This class supports display of a grid graph. The node location on canvas
# is included as a data field of the graph, graph.node_display_locations.
##############

class GridGraphDisplay(object):
    def __init__(self, frame, graph):
        self.node_dist = 80
        self.node_size = 40
        self.gui_root = frame
        self.canvas = None
        self.graph = graph
        self.nodes_location = graph.node_display_locations
        self.start_node = graph.startNode
        self.goal_node = graph.goalNode
        self.DriveCommands= []
    
        return

    # draws nodes and edges in a graph
    def display_graph(self):
        self.canvas=tk.Canvas(self.gui_root, width = '250', height = '500')
        self.canvas.pack(expand=1, fill='both')
         # print("self.graph.nodes" + str(self.graph.nodes))
        for node in self.graph.nodes:
            # print("node "+ node)
            # print("neighbors "+ str(self.graph.nodes[node]))
            # print ("neighbor: " + str(self.graph.nodes[node]))
            n1String= node
           # print(n1String + "n1String")
            n1Coords= list()
            n1Coords.append(n1String[0])
            n1Coords.append(n1String[-1])
           # print(str(n1Coords) + "n1")
            for neighbor in self.graph.nodes[node]:
                # print("neighbor")
                # print("neighbor"+ neighbor)
                n2String= neighbor
               # print(n2String + "n2String")
                n2Coords= list()
                n2Coords.append(n2String[0])
                n2Coords.append(n2String[-1])
                #print(str(n2Coords) + "n2")
                self.draw_edge(n1Coords, n2Coords , 'black')
        for node in self.nodes_location:
            self.draw_node(node, 'blue')
        startString= self.start_node

        bfs = BFS(self.graph.nodes)
        allPaths=bfs.bfs_paths(self.start_node, self.goal_node)
        shortestPath=bfs.shortest(allPaths)
        self.highlight_path(shortestPath) 
        self.DriveCommands=bfs.driveLogic(shortestPath)


        startCoords= list()
        startCoords.append(self.start_node[0])
        startCoords.append(self.start_node[-1])
        self.draw_node(startCoords, 'red')
        goalString= self.goal_node
        goalCoords= list()
        goalCoords.append(self.goal_node[0])
        goalCoords.append(self.goal_node[-1])
        self.draw_node(goalCoords, 'green')


               
    # path is a list of nodes ordered from start to goal node
    def highlight_path(self, path):
        for node in path:
            self.draw_node(node, 'yellow')
  
    # draws a node in given color. The node location info is in passed-in node object
    def draw_node(self, node, n_color):
        x=int(node[0])
        y=self.graph.grid_rows-int(node[-1])
        self.canvas.create_oval((x*self.node_dist)+20, (y*self.node_dist)+20, (x*self.node_dist)+self.node_size+20, (y*self.node_dist)+self.node_size+20, width=2, fill = n_color)
        self.canvas.create_text((x*self.node_dist)+40, (y*self.node_dist)+40, text=(str(node[0]) +  ", " + str(node[-1])))
        # self.canvas.create_oval(5, 5, 15, 15, width=2, fill = 'blue')
        self.canvas.pack()

    # draws an line segment, between two given nodes, in given color
    def draw_edge(self, node1, node2, e_color):
        x1=int(node1[0])
        y1=self.graph.grid_rows-int(node1[1])
        #print str(x1) + ',' +str(y1)
        x2=int(node2[0])
        y2=self.graph.grid_rows-int(node2[1])
        #print str(x2) + ',' +str(y2) 
        # print"line"
        self.canvas.create_line((x1*self.node_dist)+40, (y1*self.node_dist)+40, (x2*self.node_dist)+40, (y2*self.node_dist)+40, fill=e_color, width=3)
          
# def main():
#     root = tk.Tk()
#     app = tk.Frame()
#     canvas = tk.Canvas(app)
#     frame = tk.Frame(app)
#     gui= GridGraphDisplay(frame, graph)
#     gui.display_graph()
#     frame.mainloop()

# if __name__ == "__main__":
#     main()