'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   Name:          bfs_engine.py
   By:            Qin Chen
   Last Updated:  6/10/18

   PROPRIETARY and CONFIDENTIAL
   ========================================================================*/
'''
import sys

class BFS(object):
    def __init__(self, graph):
        self.graph = graph
        return

    ######################################################
    # this function returns the shortest path for given start and goal nodes
    ######################################################
    def bfs_shortest_path(self, start, goal):
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

    ######################################################
    # this function returns all paths for given start and goal nodes
    ######################################################
    def bfs_paths(self, start, goal):
        paths = []
        stack=[(start, [start])]
        while stack:
            (node, path)=stack.pop()
            # print'\n visiting node', node, 'path = ', path
            for next in self.graph[node] - set(path):
                # print 'next node', next
                if next == goal:
                    paths.append(path + [next])
                else:
                    stack.append((next, path + [next]))
                    # print "stack push ", next, path + [next]
        return paths
                
    #########################################################
    # This function returns the shortest paths for given list of paths
    #########################################################
    def shortest(self, paths):
        currentSmallest = len(paths[0])
        shortestPath= paths[0]
        for path in paths:
            if len(path)<currentSmallest:
                currentSmallest= len(path)
                shortestPath = path
        return shortestPath


    #########################################################
    # THis function traverses the graph from given start node
    # return order of nodes visited
    #########################################################
    def bfs(self, start):
        visited_order = list()
        visited = set()
        q = list([start])

        while q:
            node = q.pop(0)
            if node not in visited:
                # print("visiting", node)
                visited.add(node)
                visited_order.append(node)
                # print("--visited_order", visited_order)
                q.extend(self.graph[node]-visited)
        return visited_order

    def driveLogic(self, path):
        directions=[]
        for i in range(0,len(path)-1):
            if path[i][0]> path[i+1][0]:
                directions.append('left')
            elif path[i][0]< path[i+1][0]:
                directions.append('right')
            elif path[i][-1]> path[i+1][-1]:
                directions.append('down')
            elif path[i][-1]< path[i+1][-1]:
                directions.append('up')
        return directions


def main():
    graph = {'A': set(['B', 'C']),
         'B': set(['A', 'E', 'D']),
         'C': set(['A', 'F', 'G']),
         'D': set(['B', 'H']),
         'E': set(['B','I', 'J']),
         'F': set(['C','K']),
         'G': set(['C']),
         'H': set(['D', 'I']),
         'I': set(['E']),
         'J': set(['E']),
         'K': set(['F'])}

    bfs = BFS(graph)

    start_node = 'A'
    end_node = 'I'
    
    allPaths =bfs.bfs_paths(start_node, end_node)
    print "\n##########all paths", allPaths
    print "\n##########shortest:", bfs.shortest(allPaths)
    shortestPath=bfs.shortest(allPaths)
    # driveLogic(shortestPath)
    # print driveLogic
    return

if __name__ == "__main__":
    sys.exit(main())