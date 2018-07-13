import Queue
import sys

class Node:
    def __init__(self):
      self.name = ''
      self.data =[]
      self.f_cost = 0
      self.g_cost = sys.maxint #cost so far
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

    def h_calculator(self, a, b):
        x1 = a.data[0]
        y1 = a.data[1]
        x2 = b.data[0]
        y2 = b.data[1]
        h = abs(x1 - x2) + abs(y1 - y2)
        return h

    def getNeighbors(self, node):
        return self.nodes[node].edges #(destination, weight)

    def A_Star(self):
        # RULE: F=G+H
        print "A* Search"
        print "Start: ", self.startNode
        print "Goal: ", self.goalNode
        self.queue.put(self.startNode, 0)
        path = {}
        path[self.startNode] = None
        self.nodes[self.startNode].g_cost = 0

        while not self.queue.empty():
            current= self.queue.get()
            if current == self.goalNode:
                path[next[0]] = current
                break
            for next in self.getNeighbors(current):
                new_g_cost = self.nodes[current].g_cost + next[1]
                if (self.nodes[next[0]].g_cost is None) or (new_g_cost < self.nodes[next[0]].g_cost):
                    self.nodes[next[0]].g_cost = new_g_cost
                    self.nodes[next[0]].h_cost = self.h_calculator(self.nodes[self.goalNode], self.nodes[next[0]])
                    priority = new_g_cost + self.nodes[next[0]].h_cost
                    self.queue.put(next[0], priority)
                    path[next[0]] = current
        key = self.goalNode
        stack=Queue.LifoQueue()
        stack.put(key)
        while path[key] is not None:
            stack.put(path[key])
            key= path[key]
        i=0
        paths = []
        while not stack.empty():
            paths.append(stack.get())
            i=i+1
        print"Finish A* Search"
        return paths
      
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
      
    # def Floyd(self):
    #   #KEY:
    #   #distances[starting node][ending note][0]=the distance
    #   #distances[starting node][ending note][1]=the path
    #   #neighbor[0] = end node
    #   #neighbor[1] = weight
    #   print "Floyd Search"
    #   print "Start: ", self.startNode
    #   print "Goal: ", self.goalNode
    #   n=len(self.nodes) #number of nodes 
    #   distances=[] #array of minimum distance between each node
    #   for node in self.nodes: 
    #     if(self.nodes[node].name=='s'):
    #       self.nodes[node].name=0
    #     elif(self.nodes[node].name=='g'):
    #       self.nodes[node].name=n
    #     self.nodes[node].name= int(self.nodes[node].name)
    #     print("index: " + str(self.nodes[node].name))
    #     distances[self.nodes[node].name][self.nodes[node].name][0]= 0
    #     for neighbor in self.getNeighbors(self.nodes[node]):
    #       distances[self.nodes[node].name][neighbor[0]][0]= neighbor[1]
    #     for a in range(0,n):
    #       for b in range(0,n):
    #         for c in range(0,n):
    #           if distances[b][c][0]>distances[b][a][0]+distances[a][c][0]:
    #             distances[b][c][0]= distances[b][a][0]+distances[a][c][0]
    #             distances[b][c][1].append(self.nodes[a])
    #   return self.shortestFromFloyd(distances)

    # def shortestFromFloyd(self, distances):
    #   path=distances[self.nodes[self.startNode].name][self.nodes[self.goalNode].name][1]
    #   distance=distances[self.nodes[self.startNode].name][self.nodes[self.goalNode].name][0]
    #   return path
    
    def BFS(self):
        print "BFS Search"
        print "Start: ", self.startNode
        start = self.startNode
        print "Goal: ", self.goalNode
        goal = self.goalNode
        stack=[(start, [start])]
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

    #Assumes no diagonal movement
    def Gradient_Descent(self, vWorld, start, goal):
      grid = []
      for x in range(-vWorld.canvas_width, vWorld.canvas_width+1):
        grid.append([])
        for y in range(-vWorld.canvas_height, vWorld.canvas_height+1):
          grid[x].append([])

      q = Queue.Queue()
      q.put(goal)
      dist = 0
      #Brushfire algorithm
      while not q.empty():
        cell = q.get()
        grid[cell[0]][cell[1]] = dist
        if cell[0] - 1 >= -vWorld.canvas_width and grid[cell[0]-1][cell[1]] == []:
          q.put([cell[0] - 1, cell[1]])
        if cell[0] + 1 <= vWorld.canvas_width and grid[cell[0]+1][cell[1]] == []:
          q.put([cell[0] + 1, cell[1]])
        if cell[1] - 1 >= -vWorld.canvas_height and grid[cell[0]][cell[1]-1] == []:
          q.put([cell[0], cell[1]] - 1)
        if cell[1] + 1 <= vWorld.canvas_height and grid[cell[0]][cell[1]+1] == []:
          q.put([cell[0], cell[1]] + 1)
        dist += 1
      #Ensuring obstacles will be the highest around
      #POSSIBLE ISSUE: INCREMENTING HEIGHT MORE THAN ONCE
      obsH = dist+10
      for cobs in vWorld.cobs:
        xVals = [cobs[0], cobs[2]]
        yVals = [cobs[1], cobs[3]]
        for x in range(min(xVals), max(xVals) + 1):
          for y in range(min(yVals), max(yVals) + 1):
            grid[x][y] = obsH
            if x - 1 >= -vWorld.canvas_width:
              grid[x-1][y] += 5
            if x + 1 <= vWorld.canvas_width:
              grid[x+1][y] += 5
            if y - 1 >= -vWorld.canvas_height:
              grid[x][y-1] += 5
            if y + 1 <= vWorld.canvas_height:
              grid[x][y+1] += 5
      pos = start
      path = [start]
      while not pos == goal:
        smallest = pos
        x = pos[0]
        y = pos[1]
        if x - 1 >= -vWorld.canvas_width and grid[x-1][y] < grid[smallest[0]][smallest[1]]:
          smallest = [x-1, y]
        if x + 1 <= vWorld.canvas_width and grid[x+1][y] < grid[smallest[0]][smallest[1]]:
          smallest = [x+1, y]
        if y - 1 >= -vWorld.canvas_height and grid[x][y-1] < grid[smallest[0]][smallest[1]]:
          smallest = [x, y-1]
        if y + 1 <= vWorld.canvas_height and grid[x][y+1] < grid[smallest[0]][smallest[1]]:
          smallest = [x, y+1]
        if pos == smallest:
          break
        path.append[smallest]
        pos = smallest









