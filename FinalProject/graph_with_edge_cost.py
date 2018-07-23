import Queue
import math
import time
import copy
import sys

class Node:
	def __init__(self):
		self.name = ''
		self.data =[]
		self.f_cost = 0
		self.h_cost = 0
		self.back_pointer = False
		self.closed = False
		self.edges = []
		self.g_cost = sys.maxint #cost so far
		self.neighbors = []

class Graph:
	def __init__(self):
		#txt name to node obj
		self.nodes = {}
		self.startNode = None
		self.goalNode = None
		self.queue = Queue.PriorityQueue()
		self.allPaths = []

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

	#Assumes no diagonal movement
	def Gradient_Descent(self, vWorld, start, goal):
	  grid = []
	  for x in range(-vWorld.canvas_width, vWorld.canvas_width+1):
		grid.append([])
		x2 = x + vWorld.canvas_width
		for y in range(vWorld.canvas_height, -vWorld.canvas_height-1, -1):
		  grid[x2].append([])

	  q = Queue.Queue()
	  dist = 0
	  grid[goal[0]+vWorld.canvas_width][vWorld.canvas_height-goal[1]] = [dist]
	  q.put([goal[0]+vWorld.canvas_width, vWorld.canvas_height-goal[1]])
	  maxDist = dist
	  #Brushfire algorithm
	  maxDist = dist

	  for cobs in vWorld.cobs:
		xVals = [cobs[0], cobs[2]]
		yVals = [cobs[1], cobs[3]]
		for x in range(min(xVals), max(xVals) + 1):
		  x2 = x + vWorld.canvas_width
		  for y in range(max(yVals), min(yVals) - 1, -1):
			y2 = vWorld.canvas_height - y
			grid[x2][y2] = 'No'


	  while not q.empty():
		cell = q.get()

		if not grid[cell[0]][cell[1]] == 'No':


			#Find lowest cost of all that have been appended
			lowest = None
			for vals in grid[cell[0]][cell[1]]:
			  if lowest is None or vals < lowest:
				lowest = vals
			grid[cell[0]][cell[1]] = lowest


			#Add possible costs to adjacent cells and add said cells to the queue
			#Vertical and horizontal

			dist = grid[cell[0]][cell[1]] + 1
			if cell[0] - 1 >= 0 and grid[cell[0]-1][cell[1]] == []:
			  grid[cell[0]-1][cell[1]].append(dist)
			  q.put([cell[0] - 1, cell[1]])
			if cell[0] + 1 <= vWorld.canvas_width*2 and grid[cell[0]+1][cell[1]] == []:
			  grid[cell[0]+1][cell[1]].append(dist)
			  q.put([cell[0] + 1, cell[1]])
			if cell[1] - 1 >= 0 and grid[cell[0]][cell[1]-1] == []:
			  grid[cell[0]][cell[1]-1].append(dist)
			  q.put([cell[0], cell[1] - 1])
			if cell[1] + 1 <= vWorld.canvas_height*2 and grid[cell[0]][cell[1]+1] == []:
			  grid[cell[0]][cell[1]+1].append(dist)
			  q.put([cell[0], cell[1] + 1])
			#Diagonal
			if cell[0] - 1 >= 0 and cell[1] - 1 >= 0 and grid[cell[0]-1][cell[1]-1] == []:
			  grid[cell[0]-1][cell[1]-1].append(math.sqrt(2) + dist - 1)
			  q.put([cell[0] - 1, cell[1] - 1])
			if cell[0] + 1 <= vWorld.canvas_width*2 and cell[1] + 1 <= vWorld.canvas_height*2 and grid[cell[0]+1][cell[1]+1] == []:
			  grid[cell[0]+1][cell[1]+1].append(math.sqrt(2) + dist - 1)
			  q.put([cell[0] + 1, cell[1] + 1])
			if cell[0] - 1 >= 0 and cell[1] + 1 <= vWorld.canvas_height*2 and grid[cell[0]-1][cell[1]+1] == []:
			  grid[cell[0]-1][cell[1]+1].append(math.sqrt(2) + dist - 1)
			  q.put([cell[0] - 1, cell[1] + 1])
			if cell[0] + 1 <= vWorld.canvas_width*2 and cell[1] - 1 >= 0 and grid[cell[0]+1][cell[1]-1] == []:
			  grid[cell[0]+1][cell[1]-1].append(math.sqrt(2) + dist - 1)
			  q.put([cell[0]+1, cell[1] - 1])
			#Keeps track of the highest cost
			if dist > maxDist:
			  maxDist = dist
	  

	  #Ensuring obstacles will be the highest around
	  obsH = maxDist+10
	  for cobs in vWorld.cobs:
		xVals = [cobs[0], cobs[2]]
		yVals = [cobs[1], cobs[3]]
		for x in range(min(xVals), max(xVals) + 1):
		  x2 = x + vWorld.canvas_width
		  for y in range(max(yVals), min(yVals) - 1, -1):
			y2 = vWorld.canvas_height - y
			grid[x2][y2] = obsH
			'''
			if x - 1 >= -vWorld.canvas_width and not grid[x2-1][y2] == obsH:
			  grid[x2-1][y2] += 5
			if x + 1 <= vWorld.canvas_width and not grid[x2+1][y2] == obsH:
			  grid[x2+1][y2] += 5
			if y - 1 >= -vWorld.canvas_height and not grid[x2][y2-1] == obsH:
			  grid[x2][y2-1] += 5
			if y + 1 <= vWorld.canvas_height and not grid[x2][y2+1] == obsH:
			  grid[x2][y2+1] += 5
			'''
	  #print 'modded obstacles'
	  #print obsH
	  '''
	  for y in range(vWorld.canvas_height, -vWorld.canvas_height-1, -1):
		y2 = vWorld.canvas_height - y
		string = ''
		for x in range(-vWorld.canvas_width, vWorld.canvas_width+1):
		  x2 = x + vWorld.canvas_width
		  if [x, y] == start:
			string += 'sSs'
		  else:
			pstring = str(grid[x2][y2])
			if len(pstring) > 3:
			  string += pstring[0] + pstring[1] + pstring[2]
			elif len(pstring) == 2:
			  string += str(0) + pstring[0] + pstring[1]
			else:
			  string += str(0) + str(0) + pstring[0]
		  string += ' '
		print string
	  '''
	  pos = start
	  #pos = [start[0]+vWorld.canvas_width, vWorld.canvas_height-start[1]]
	  path = [pos]
	  while not pos == goal:
		#print 'ran'
		#smallest = pos
		x = pos[0] + vWorld.canvas_width
		y = vWorld.canvas_height - pos[1]
		smallest = [[x, y], pos]
		if x - 1 >= 0 and grid[x-1][y] <= grid[smallest[0][0]][smallest[0][1]]:
		  smallest = [[x-1, y], [pos[0]-1, pos[1]]]
		if x + 1 <= vWorld.canvas_width*2 and grid[x+1][y] <= grid[smallest[0][0]][smallest[0][1]]:
		  smallest = [[x+1, y], [pos[0]+1, pos[1]]]
		if y - 1 >= 0 and grid[x][y-1] <= grid[smallest[0][0]][smallest[0][1]]:
		  smallest = [[x, y-1], [pos[0], pos[1]+1]]
		if y + 1 <= vWorld.canvas_height*2 and grid[x][y+1] <= grid[smallest[0][0]][smallest[0][1]]:
		  smallest = [[x, y+1], [pos[0], pos[1]-1]]
		
		if x - 1 >= 0 and y - 1 >= 0 and grid[x-1][y-1] <= grid[smallest[0][0]][smallest[0][1]]:
		  smallest = [[x-1, y-1], [pos[0]-1, pos[1]+1]]
		if x + 1 <= vWorld.canvas_width*2 and y + 1 <= vWorld.canvas_height*2 and grid[x+1][y+1] <= grid[smallest[0][0]][smallest[0][1]]:
		  smallest = [[x+1, y+1], [pos[0]+1, pos[1]-1]]
		if y - 1 >= 0 and x + 1 <= vWorld.canvas_width*2 and grid[x+1][y-1] <= grid[smallest[0][0]][smallest[0][1]]:
		  smallest = [[x+1, y-1], [pos[0]+1, pos[1]+1]]
		if y + 1 <= vWorld.canvas_height*2 and x - 1 >= 0 and grid[x-1][y+1] <= grid[smallest[0][0]][smallest[0][1]]:
		  smallest = [[x-1, y+1], [pos[0]-1, pos[1]-1]]
		
		if pos == smallest[1]:
		  break
		path.append(smallest[1])
		pos = smallest[1]
	  #print 'returned path'
	  return path

	def BFS(self):
		return self.bfs_shortest_path(self.startNode, self.goalNode)

	def bfs_shortest_path(self, start, goal):
		#Used to store the next node
		q = Queue.Queue()
		q.put([start, [], set(), 0])
		self.traverse(q, goal)
		allPaths = copy.copy(self.allPaths)
		self.allPaths = []
		if len(allPaths) >= 1:
			shortest = self.find_shortest(allPaths)
			return shortest[0]
		else:
			return []

	def traverse(self, q, goal):
		#Until every path has been tested
		while not q.empty():
			info = q.get()
			#The name
			node = info[0]
			#The current path
			path = info[1]
			#The nodes visited by the path
			visited = info[2]
			#The current cost of the path
			curCost = info[3]
			#Add the current node to both the path and visited
			path.append(node)
			visited.add(node)
			#Checking all adjacent, unvisited nodes
			for n in self.nodes[node].edges:
				if not n[0] in visited:
					#Calculate cost to adjacent node
					newCost = curCost + n[1]
					if(n[0] == goal):
						end_path = copy.copy(path)
						end_path.append(n[0])
						#Add to list of valid paths
						self.allPaths.append([end_path, newCost])
					else:
						q.put([n[0], list(path), set(visited), newCost])

	def find_shortest(self, paths):
		shortest = None
		for i in range(0, len(paths)):
			path = paths[i][0]
			total = paths[i][1]
			print 'Simon says', path, total
			if shortest is None or shortest[1] > total:
				shortest = [path, total]
		return shortest

	def DFS(self):
		return self.dfs_shortest_path(self.startNode, self.goalNode)

	def dfs_shortest_path(self, start, goal):
		#Used to store the next node
		#Using stack means depth first, not breadth first
		q = Queue.LifoQueue()
		q.put([start, [], set(), 0])
		self.traverse_dfs(q, goal)
		allPaths = copy.copy(self.allPaths)
		self.allPaths = []
		if len(allPaths) >= 1:
			shortest = self.find_shortest(allPaths)
			return shortest[0]
		else:
			return []

	def traverse_dfs(self, q, goal):
		#Until every path has been tested
		while not q.empty():
			info = q.get()
			#The name
			node = info[0]
			#The current path
			path = info[1]
			#The nodes visited by the path
			visited = info[2]
			#The current cost of the path
			curCost = info[3]
			#Add the current node to both the path and visited
			path.append(node)
			visited.add(node)
			#Checking all adjacent, unvisited nodes
			for n in self.nodes[node].edges:
				if not n[0] in visited:
					#Calculate cost to adjacent node
					newCost = curCost + n[1]
					if(n[0] == goal):
						end_path = copy.copy(path)
						end_path.append(n[0])
						#Add to list of valid paths
						self.allPaths.append([end_path, newCost])
					else:
						q.put([n[0], list(path), set(visited), newCost])

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
		print "A* Search"
		print "Start: ", self.startNode
		print "Goal: ", self.goalNode
		self.queue.put(self.startNode, 0)
		path = {} #create a dictionary to hold the key of each node and value of shortest path to it
		path[self.startNode] = None #no node leads to the start node
		self.nodes[self.startNode].g_cost = 0 #the cost to get from start to start = 0
		while not self.queue.empty(): 
			current= self.queue.get()
			if current == self.goalNode:
				break #exit loop when we reach the goal node
			for neighbor in self.getNeighbors(current):
				new_g_cost = self.nodes[current].g_cost + neighbor[1] #cost to get from start to the previous + new edge weight
				if (self.nodes[neighbor[0]].g_cost is None) or (new_g_cost < self.nodes[neighbor[0]].g_cost):
				  #if this new g_cost is the best option^
					self.nodes[neighbor[0]].g_cost = new_g_cost
					#estimate cost from new node to finish by calculating the distance
					self.nodes[neighbor[0]].h_cost = self.h_calculator(self.nodes[self.goalNode], self.nodes[neighbor[0]])
					priority = new_g_cost + self.nodes[neighbor[0]].h_cost
					self.queue.put(neighbor[0], priority)
					path[neighbor[0]] = current #set shortest path to the neighbor to the value of the current node
		key = self.goalNode
		stack=Queue.LifoQueue()
		stack.put(key)
		while path[key] is not None:
			stack.put(path[key])
			print key
			key= path[key] #follow the trail of paths from goal to start and add them to a stack
		i=0
		paths = []
		while not stack.empty():
			paths.append(stack.get()) #add the values of the stack to this list in order start to goal
			i+=1
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
		distances=[[[sys.maxint,[]] for x in range(n+1)] for y in range(n+1)]  
		#array of minimum distance between each node (start each length equal to max-int and each path as empty)
		#so start and goal can be referenced as integers:
		self.nodes['0'] = self.nodes.pop("s") 
		self.nodes[str(n)] = self.nodes.pop("g")
		self.nodes['0'].name=0
		self.nodes[str(n)].name=n
		for node in self.nodes:
			distances[int(self.nodes[node].name)][int(self.nodes[node].name)][0] = 0 #distance from any node to itself is 0
			for neighbor in self.getNeighborsFloyd(node, n): #distance between neighbors is just the edge weight
				distances[int(self.nodes[node].name)][int(self.nodes[neighbor[0]].name)][0] = neighbor[1] 
		for a in range(0,n+1):
			for b in range(0,n+1):
				for c in range(0,n+1):
					if distances[b][c][0]>distances[b][a][0]+distances[a][c][0]: #if a path is more efficient than the existing one
						distances[b][c][0] = distances[b][a][0]+distances[a][c][0] #replace the existing path with the optimal one
						distances[b][c][1] = [self.nodes[str(a)]]
		return self.shortestFromFloyd(distances, 0, n) #get the shortest path from start to goal from the matrix of all shortest paths

	def getNeighborsFloyd(self, node, n):
		adjs = self.nodes[node].edges #(destination, weight)
		i = 0
		#again, change s and g so they can be referenced as integers:
		while i < len(adjs):
			if adjs[i][0] == 's':
				adjs[0][0] = '0'
			elif adjs[i][0] == 'g':
				adjs[i][0] = str(n)
			i += 1
		return adjs

	def shortestFromFloyd(self, distances, start, goal):
		path=distances[start][goal][1] + [self.nodes[str(goal)]] #path from start to goal
		while not distances[int(path[0].name)][goal][1] == []: #until you reach the start node
			path = distances[int(path[0].name)][goal][1] + path #add next detour
		path = [self.nodes[str(start)]] + path
		return path

	def djk(self):
		#Create a priority queue to sort the possible next nodes
		q = Queue.PriorityQueue()
		q.put(self.startNode, 1)
		while not q.empty():
			name = q.get()
			node = self.nodes[name]
			cost  = node.f_cost
			#Make sure not to revisit
			node.closed = True
			for e in node.edges:
				#get the actual node object
				edge = self.nodes[e[0]]
				if not edge.closed:
					#If you've reached the goal
					if edge == self.nodes[self.goalNode]:
						edge.back_pointer = node
						break
					newCost = cost + e[1]
					#In case you've found a new shortest path
					if edge.f_cost == 0 or edge.f_cost > newCost:
						edge.f_cost = newCost
						edge.back_pointer = node
						q.put(edge.name, newCost)

