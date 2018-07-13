import Queue
import sys

class Node:
    def __init__(self):
      self.name = ''
      self.data =[]
      self.f_cost = 0
      self.g_cost = 0 #cost so far
      self.h_cost = 0 #estimate from goal to node
      self.back_pointer = False
      self.closed = False
      self.edges = []
      self.neighbors = []

class Graph:
    def __init__(self):
        self.nodes = {}
        self.startNode = None
        self.goalNode = None
        self.queue = Queue.PriorityQueue()

    def add_node(self, name, data):
        a_node = Node()
        a_node.name = name
        a_node.data = data
        self.nodes[name] = a_node
        return a_node

    def set_start(self, name):
        self.startNode = name
        self.nodes[name].f_cost = 0
        self.queue.put((0, self.nodes[name]))

    def set_goal(self, name):
        self.goalNode = name

    def add_edge(self, node1, node2, g_cost):
        self.nodes[node1].edges.append([node2, g_cost])
        self.nodes[node2].edges.append([node1, g_cost])
    
    def getNeighbors(self, node):
        return self.edges[node] #(destination, weight)

    def Dijkstra(self):
      print "Dijkstra's Greedy Search"
      print "Start: ", self.startNode
      print "Goal: ", self.goalNode
      #print "Queue: ", self.queue

      while not self.queue.empty():
        current_node = self.queue.get()[1]
        #print "current node: ", current_node.name, current_node.f_cost
        for an_edge in current_node.edges:
          a_node_name = an_edge[0]
          if not self.nodes[a_node_name].closed:
            #print "expand next node: ", a_node_name
            f_cost = current_node.f_cost + an_edge[1]
            if not self.nodes[a_node_name].f_cost or self.nodes[a_node_name].f_cost > f_cost:
                self.nodes[a_node_name].f_cost = f_cost
                self.nodes[a_node_name].back_pointer = current_node
                self.queue.put((self.nodes[a_node_name].f_cost, self.nodes[a_node_name]))
                #print "put queue: ", a_node_name, f_cost
                if a_node_name == self.goalNode:
                  #print "found path with cost: ", self.nodes[a_node_name].f_cost
                  #print "path node: ", a_node_name
                  path_node = self.nodes[a_node_name]
                  while path_node.back_pointer != False:
                    path_node = path_node.back_pointer
                    #print "path node: ", path_node.name
        current_node.closed = True

      def h_calculator(a, b):
          (x1, y1) = a
          (x2, y2) = b
          h = abs(x1 - x2) + abs(y1 - y2)
          return h

      def aStar(self, start, goal):
          # RULE: F=G+H
          print "A* Search"
          print "Start: ", self.startNode
          print "Goal: ", self.goalNode
          self.queue.put(start, 0)
          path = {}
          path[start] = None
          start.g_cost = 0
          while not self.queue.empty():
              current = self.queue.get()
              if current == goal:
                  break
              for next in self.getNeighbors(current):
                  new_g.cost = current.g_cost + next[1]
                  if next not in next.g_cost or new_g.cost < next.g_cost:
                      next.g_cost = new_g.cost
                      next.h_cost = h_calculator(goal, next)
                      priority = new_g.cost + h_cost
                      self.queue.put(next, priority)
                      path[next] = current
          return path

      def Floyd(self):
        #KEY:
        #distances[starting node][ending note][0]=the distance
        #distances[starting node][ending note][1]=the path
        #neighbor[0] = end node
        #neighbor[1] = weight
        print "Floyd Search"
        print "Start: ", self.startNode
        print "Goal: ", self.goalNode
        n=len(self.nodes) #number of nodes 
        distances=[n][n] #array of minimum distance between each node
        for node in self.nodes: 
          distances[node][node][0]= 0
          for neighbor in self.getNeighbors(node):
            distances[node][neighbor[0]][0]= neighbor[1]
          for a in range(0,n):
            for b in range(0,n):
              for c in range(0,n):
                if distances[b][c][0]>distances[b][a][0]+distances[a][c][0]:
                  distances[b][c][0]= distances[b][a][0]+distances[a][c][0]
                  distances[b][c][1].append(self.nodes[a])
        return shortestFromFloyd(distances)

      def shortestFromFloyd(distances, start, goal):
        path=distances[start][goal][1]
        distance=distances[start][goal][0]
        return path
      
      def bfs(self, start, goal):
          this=[(start, [start])]
          while stack:
              (node, path)=stack.pop[0]
              # print '\n visiting node', node, 'path = ', path
              for next in (self.graph[node] - set(path)):
                  # print 'next node', next
                  if next == goal:
                      return path + [next]
                  else:
                      stack.append((next, path + [next]))
                      # print "stack push ", next, path + [next]
          return
