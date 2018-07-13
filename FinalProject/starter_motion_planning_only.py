'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   PROPRIETARY and CONFIDENTIAL

   Creqted by Dr. Qin Chen
   7/2017
   ========================================================================*/
'''

import sys
sys.path.append('../')
import Tkinter as tk
from graph_with_edge_cost import *
from tk_hamster_GUI_Sim import *
import random

class MotionPlanner(object):
    def __init__(self, vWorld, start, goal):
        self.vWorld = vWorld
        self.start = start
        self.goal = goal
        return

    def worker(self):
        print'MotionPlanner is called'
        vWorld = self.vWorld
        start = self.start
        goal = self.goal
        canvas_width = vWorld.canvas_width
        canvas_height = vWorld.canvas_height
        cell_list = Queue.Queue()
        cell_list.put(vWorld.area)
        # inflate obstacles to form C-space
        self.compute_c_obstacles(vWorld,28)
        obs_list = vWorld.cobs
        vWorld.goal_list = []
        f_cell_list = []
        # Cut inflated obstacles out of C-space and divide workspace into cells from cutting 
        f_cell_list = self.compute_free_cells(cell_list, obs_list)
        # determine connectivity between free cells and locate the point to go from one cell to its neighbor
        point_list = self.compute_free_points(f_cell_list)
        
        raw_input('press RETURN to show free cells')
        for cell in f_cell_list:
            x1 = cell[0]
            y1 = cell[1]
            x2 = cell[2]
            y2 = cell[3]
            vWorld.canvas.create_rectangle(canvas_width+x1, canvas_height-y1, canvas_width+x2, canvas_height-y2, outline = "orange")
        
        raw_input('press RETURN to show start and goal')
        #create graph - nodes and edges for the point list
        myGraph = Graph()
        num_points = len(point_list)

        # creating nodes
        myGraph.add_node("s", start)
        myGraph.set_start("s")
        myGraph.add_node("g", goal)
        myGraph.set_goal("g")
        xs = start[0]
        ys = start[1]
        vWorld.canvas.create_oval(canvas_width+xs-6, canvas_height-ys-6, canvas_width+xs+6, canvas_height-ys+6, outline = "green", fill="green")
        xg = goal[0]
        yg = goal[1]
        vWorld.canvas.create_oval(canvas_width+xg-6, canvas_height-yg-6, canvas_width+xg+6, canvas_height-yg+6, outline = "purple", fill="purple")
        
        raw_input('press RETURN to show points connecting free cells, start, and goal')
        point_num = 1
        for point in point_list:
            myGraph.add_node(str(point_num), point)
            x1 = point[0]
            y1 = point[1]
            vWorld.canvas.create_oval(canvas_width+x1-4, canvas_height-y1-4, canvas_width+x1+4, canvas_height-y1+4, outline = "red")
            if self.connected(point, start, f_cell_list):
                g_cost = math.sqrt((xs-x1)*(xs-x1)+(ys-y1)*(ys-y1))
                #print "creating edge: ", "s", str(point_num), g_cost
                myGraph.add_edge("s", str(point_num), g_cost)
                vWorld.canvas.create_line(canvas_width+x1, canvas_height-y1, canvas_width+xs, canvas_height-ys, fill="black")
            if self.connected(point, goal, f_cell_list):
                g_cost = math.sqrt((xg-x1)*(xg-x1)+(yg-y1)*(yg-y1))
                #print "creating edge: ", "g", str(point_num), g_cost
                myGraph.add_edge("g", str(point_num), g_cost)
                vWorld.canvas.create_line(canvas_width+x1, canvas_height-y1, canvas_width+xg, canvas_height-yg, fill="black")
            point_num += 1

        raw_input("press RETURN to show connectivity")
        if num_points > 1:
            # creating edges
            #print "num points: ", num_points
            next_point = 2
            for i in range (1, num_points+1):
                #print "from: ", i
                point1 = point_list[i-1]
                x1 = point1[0]
                y1 = point1[1]
                for j in range (next_point, num_points+1):
                    #print "to: ", j
                    point2 = point_list[j-1]
                    x2 = point2[0]
                    y2 = point2[1]
                    if (self.connected(point1, point2, f_cell_list)):
                        g_cost = math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))
                        #print "creating edge: ", str(i), str(j), g_cost
                        myGraph.add_edge(str(i), str(j), g_cost)
                        vWorld.canvas.create_line(canvas_width+x1, canvas_height-y1, canvas_width+x2, canvas_height-y2)
                next_point += 1

        #raw_input('press RETURN to show path')
        search_methods = ['Dijkstra\'s', 'A*', 'BFS', 'Floyd', 'Gradient Descent']
        while True:
            search = raw_input('Enter the desired search method out of: Dijkstra\'s, A*, BFS, Floyd, Gradient Descent\n')
            if search in search_methods:
                break
        #print "search: ", myGraph.queue
        if search == search_methods[0]: 
            myGraph.Dijkstra()
        elif search == search_methods[1]:
            myGraph.A_Star()
        elif search == search_methods[2]:
            myGraph.BFS()
        elif search == search_methods[3]:
            myGraph.Floyd()
        elif search == search_methods[4]:
            myGraph.Gradient_Descent()

        path = Queue.LifoQueue()
        if myGraph.nodes["g"].back_pointer:
            path.put(["pose", myGraph.nodes["g"].data[0], myGraph.nodes["g"].data[1], False])
            print "Found path"
            path_node = myGraph.nodes["g"].back_pointer
            while path_node.back_pointer != False:
                px = path_node.data[0]
                py = path_node.data[1]
                path.put(["pose", px, py, False])
                vWorld.canvas.create_oval(canvas_width+px-6, canvas_height-py-6, canvas_width+px+6, canvas_height-py+6, outline = "red", fill="red")
                path_node = path_node.back_pointer
            while not path.empty():
                vWorld.goal_list.append(path.get())
                vWorld.goal_list_index = 0
            print "path: ", vWorld.goal_list
        else:
            print "failed to find path"
        return

    def compute_c_obstacles(self, vworld, d):
        # save c-space obstacle location info in vWorld.cobs[]
        #assumes x1, y1, x2, y2 top left bottom right
        for rect in vworld.map:
            vworld.cobs.append([rect[0]-d, rect[1]-d, rect[2]+d, rect[3]+d])

    def compute_free_cells(self, cell_list, c_obs_list):
        #return f_cell_list
        points = []
        
        for rect in c_obs_list:
            '''
            points.append([rect[0], rect[1]])
            points.append([rect[0], rect[3]])
            points.append([rect[2], rect[1]])
            points.append([rect[2], rect[3]])
            '''
            points.append( [min([rect[0],rect[2]]), max([rect[1], rect[3]])] )
            points.append( [min([rect[0],rect[2]]), min([rect[1], rect[3]])] )
            points.append( [max([rect[0],rect[2]]), max([rect[1], rect[3]])] )
            points.append( [max([rect[0],rect[2]]), min([rect[1], rect[3]])] )

        hLines = []
        vLines = []
        
        test_point = None
        points.append([-self.vWorld.canvas_width, -self.vWorld.canvas_height])
        i = 0
        while i <= len(points)-1:
            #vertical line

            point = points[i]

            x = point[0]
            y = self.vWorld.canvas_height
            x2 = point[0]
            y2 = point[1]

            result = self.line_intersect_cobs_prnt(x, y, x2, y2, x2, y2, False)
            '''
            if result == False:
                vLines.append([x, y, x2, y2])
            elif result == True:
                pass
            else:
                vLines.append([x, result[0], x2, result[1]])
            '''
            if result == False:
                vLines.append([x, y, x2, y2])
            elif result == True:
                pass
            else:
                if result[0] is None:
                    result[0] = y
                if result[1] is None:
                    result[1] = y2
                vLines.append([x, result[0], x2, result[1]])
            '''
            if len(vLines) == 1:
                print point
                print [x, y, x2, y2]
                print self.line_intersect_cobs_prnt(x, y, x2, y2, x2, y2, False)
                # x2, result[1]]
                print '******'
            '''
            if i+1 < len(points):
                point = points[i+1]

                x = point[0]
                y = point[1]
                x2 = point[0]
                y2 = -self.vWorld.canvas_height

                result = self.line_intersect_cobs_prnt(x, y, x2, y2, x, y, False)
                '''
                if result == False:
                    vLines.append([x, y, x2, y2])
                elif result == True:
                    pass
                else:
                    vLines.append([x, result[0], x2, result[1]])
                '''

                if result == False:
                    vLines.append([x, y, x2, y2])
                elif result == True:
                    pass
                else:
                    if result[0] is None:
                        result[0] = y
                    if result[1] is None:
                        result[1] = y2
                    vLines.append([x, result[0], x2, result[1]])
                '''
                if len(vLines) == 2:
                    test_point = point
                    print point
                    print [x, y, x2, y2]
                    print self.line_intersect_cobs_prnt(x, y, x2, y2, x, y, False)
                    #print [x, result[0], x2, result[1]]
                    print '******'
                '''
            i += 2

        #print 'here'
        cells = []

        changed = True
        while changed:
            changed = False
            for i in range(0, len(vLines)-1):
                if vLines[i][0] > vLines[i+1][0]:
                    tmp = vLines[i]
                    vLines[i] = vLines[i+1]
                    vLines[i+1] = tmp
                    changed = True


        for rect in c_obs_list:
            self.vWorld.canvas.create_rectangle(self.vWorld.canvas_width+rect[0], self.vWorld.canvas_height-rect[1], self.vWorld.canvas_width+rect[2], self.vWorld.canvas_height-rect[3], fill='', outline='red', width='1')
        #print '*******'
        #line = vLines[1]
        #print test_point
        #self.line_intersect_cobs_prnt(test_point[0], test_point[1], test_point[0], -self.vWorld.canvas_height, test_point[0], test_point[1], False)
        #print vLines[1]
        #print self.vWorld.canvas_width+line[0], self.vWorld.canvas_height-line[1], self.vWorld.canvas_width+line[2], self.vWorld.canvas_height-line[3]
        #self.vWorld.canvas.create_line(self.vWorld.canvas_width+line[0], self.vWorld.canvas_height-line[1], self.vWorld.canvas_width+line[2], self.vWorld.canvas_height-line[3], fill='blue')
        #self.vWorld.canvas.create_oval(self.vWorld.canvas_width+line[0]-2, self.vWorld.canvas_height-line[1]-2, self.vWorld.canvas_width+line[0]+2, self.vWorld.canvas_height-line[1]+2, fill='red')
        #self.vWorld.canvas.create_oval(self.vWorld.canvas_width+line[2]-2, self.vWorld.canvas_height-line[3]-2, self.vWorld.canvas_width+line[2]+2, self.vWorld.canvas_height-line[3]+2, fill='green')
        #print 'Here it is', points[len(points)-1]
        #print vLines[len(vLines)-1]
        #print '*******'
        count = 0
        for line in vLines:
            self.vWorld.canvas.create_line(self.vWorld.canvas_width+line[0], self.vWorld.canvas_height-line[1], self.vWorld.canvas_width+line[2], self.vWorld.canvas_height-line[3], fill='blue')
            self.vWorld.canvas.create_oval(self.vWorld.canvas_width+line[0]-2, self.vWorld.canvas_height-line[1]-2, self.vWorld.canvas_width+line[0]+2, self.vWorld.canvas_height-line[1]+2, fill='red')
            self.vWorld.canvas.create_oval(self.vWorld.canvas_width+line[2]-2, self.vWorld.canvas_height-line[3]-2, self.vWorld.canvas_width+line[2]+2, self.vWorld.canvas_height-line[3]+2, fill='green')

            xVals = [line[0], line[2]]
            yVals = [line[1], line[3]]

            '''
            if min(xVals) == -72:
                print 'here', count
            '''
            #sets vals of top of line
            tx = min(xVals)
            ty = max(yVals)

            by = min(yVals)

            vals = self.looking_left(line, vLines, cells, tx, ty, None, by)
            #[-93, -183, -73, -163],
            #[-45, -97, -45, -135]
            #if line[0] == -45:
            #    print 'this is right'
            #    print line
            if not vals == False:
                if line == [-45, -97, -45, -135]:
                    print 'inside left'
                    print vals
                if self.contained(vals[0], vals[1], vals[2], vals[3], cells) or self.inside_cell(vals[0], vals[1], cells) or self.contains_cobs_or_cells(vals[0], vals[1], vals[2], vals[3], cells):
                    pass
                else:
                    cells.append(vals)
                    count += 1

            closest = None
            #finds the closest line to the right, while also sort of making sure its not wrong
            for line2 in vLines:
                xVals = [line2[0], line2[2]]
                yVals = [line2[1], line2[3]]
                #if (closest is None and max(xVals)-tx > 0) or (min(yVals) == by and max(xVals)-tx > 0 and abs(max(xVals)-tx) < abs(min([closest[0], closest[2]])-tx)):
                #    closest = line2

                '''
                if (closest is None and max(xVals)-tx > 0) or (max(xVals)-tx > 0 and abs(max(xVals)-tx) < abs(min([closest[0], closest[2]])-tx) and max(yVals) > max([closest[1], closest[3]])):
                    closest = line2
                '''
                #Finds the closest line to the right of the current line whose y can be higher as long as inside rect up is false for the new line, or in the second case the old line
                if (closest is None and max(xVals)-tx > 0) or (max(xVals)-tx > 0 and abs(max(xVals)-tx) < abs(min([closest[0], closest[2]])-tx)):
                    if (not self.inside_rect_up(xVals[0], max(yVals)) or max(yVals) >= ty) and (not self.inside_rect_up(tx, ty) or (self.inside_rect_up(tx, ty) and max(yVals) <= ty)) and not self.line_inbetween(tx, line2[0], vLines):
                        closest = line2
            added = False
            #if line[1] == -212:
            #    print 'this is right', line
            #[-228, -212, -228, -300]
            #[-152, -212, -152, -300]
            if closest is None:
                #Finds next cobs to the right
                obs = self.find_next_cobs(tx, ty, cells, vLines)
                if obs is None:

                    if line == [-152, -212, -152, -300]:
                        print 'obs is none'

                    bx = self.vWorld.canvas_width
                    
                    moreToRight = self.find_next_line_right(tx, ty, vLines)
                    #if no more lines to right
                    if moreToRight is None:
                        if self.contained(tx, ty, bx, by, cells) or self.inside_cell(tx, ty, cells) or self.contains_cobs_or_cells(tx, ty, bx, by, cells):
                        #if self.contained(tx, ty, bx, by, cells) or self.inside_cell(tx, ty, cells):
                            pass
                        else:
                            #stretch to fill remaining space
                            ty = self.vWorld.canvas_height
                            by = -self.vWorld.canvas_height
                            added = True
                            print '1 append'
                            cells.append([tx, ty, bx, by])
                            count += 1
                            #print 'Added sneakily'
                    else:
                        #assumes there are some obstacles in the way, finds them
                        obs2 = self.find_next_cobs_up(tx, ty, cells)
                        if obs2 is None:
                            ty = self.vWorld.canvas_height
                        else:
                            ty = min([obs2[1], obs2[3]])
                        obs2 = self.find_next_cobs_down(tx, ty, cells)
                        if obs2 is None:
                            by = -self.vWorld.canvas_height
                        else:
                            by = max([obs2[1], obs2[3]])
                        bx = moreToRight[0]
                else:

                    if line == [-152, -212, -152, -300]:
                        print 'obs is', obs
                        print count
                        xVals = [obs[0], obs[2]]
                        yVals = [obs[1], obs[3]]
                        self.vWorld.canvas.create_rectangle(self.vWorld.canvas_width+min(xVals), self.vWorld.canvas_height-max(yVals), self.vWorld.canvas_width+max(xVals), self.vWorld.canvas_height-min(yVals), fill='', outline='black', width=2)

                    #assumes there are some obstacles in the way, finds them
                    obs2 = self.find_next_cobs_up(tx, ty, cells)
                    
                    if line == [-152, -212, -152, -300]:
                        print 'obs2:', obs2
                    
                    if obs2 is None:
                        ty = self.vWorld.canvas_height
                    else:
                        ty = min([obs2[1], obs2[3]])
                    obs2 = self.find_next_cobs_down(tx, ty, cells)
                    
                    if line == [-152, -212, -152, -300]:
                        print 'obs3:', obs2
                    
                    if obs2 is None:
                        by = -self.vWorld.canvas_height
                    else:
                        by = max([obs2[1], obs2[3]])
                    bx = min([obs[0], obs[2]])
            else:
                if line == [-152, -212, -152, -300]:
                    print 'closest is'
                    print 'closest:', closest
                #In the case of multiple lines all with the same x, finds the bottom most valid one
                while True:
                    #Move down until you either hit the bottom of the screen, or enter an obstacle
                    xVals = [closest[0], closest[2]]
                    yVals = [closest[1], closest[3]]
                    if self.inside_rect_down(xVals[0], min(yVals)):
                        bx = closest[2]
                        by = closest[3]
                        break

                    bx = closest[2]

                    nLine = self.find_next_line(xVals[0], min(yVals), vLines)
                    if nLine is None:
                        break
                    closest = nLine
                if line == [-152, -212, -152, -300]:
                    print 'closest', closest
                if min([closest[1], closest[3]]) > min([line[1], line[3]]):
                    by = min([line[1], line[3]])
                else:
                    by = closest[3]

            if self.contained(tx, ty, bx, by, cells) or self.inside_cell(tx, ty, cells) or added or self.contains_cobs_or_cells(tx, ty, bx, by, cells):
                pass
            else:
                if count == -1:
                    print 'contains cobs and cells'
                    print tx, ty, bx, by
                    #print self.contains_cobs_or_cells_prnt(tx, ty, bx, by, cells)
                cells.append([tx, ty, bx, by])
                print '2 append'
                count += 1
            
            if count == 7:
                print '*-----break-----*'
                print 'line:', line
                print 'closest:', closest
                print 'coords:', tx, ty, bx, by
                #print self.inside_cell(tx, ty, cells)
                #print self.inside_rect_up(tx, ty)
                #self.vWorld.canvas.create_rectangle(self.vWorld.canvas_width+tx, self.vWorld.canvas_height-ty, self.vWorld.canvas_width+bx, self.vWorld.canvas_height-by, fill='', outline='black', width=2)
                print '*-----end-----*'
                #print self.inside_rect_down_prnt()
            #print line
            #print tx, ty, bx, by
            #print 'added'
            #self.vWorld.canvas.create_rectangle(self.vWorld.canvas_width+tx, self.vWorld.canvas_width-ty, self.vWorld.canvas_width+bx, self.vWorld.canvas_width-by, fill='red')
        #for i in range(0, len(cells)):
        #print '------break------'
        #print len(cells)
        num = 8
        for i in range(0, num+1):
            cell = cells[i]
            print cell
            #self.vWorld.canvas.create_rectangle(self.vWorld.canvas_width+cell[0], self.vWorld.canvas_height-cell[1], self.vWorld.canvas_width+cell[2], self.vWorld.canvas_height-cell[3], fill='', outline='orange')
        #for rect in c_obs_list:
        #    yVals = [rect[1], rect[3]]
        #    print min(yVals)

        return cells

    def looking_left(self, line, lines, cells, tx, ty, bx, by):
        #Finds next cobs to the right
        obs = self.find_next_cobs_left(tx, ty, cells, lines)
        if line == [-45, -97, -45, -135]:
            print 'obs', obs
        if obs is None:
            return False
        else:
            if line == [-152, -212, -152, -300]:
                print 'obs is', obs
                print count
                xVals = [obs[0], obs[2]]
                yVals = [obs[1], obs[3]]
                self.vWorld.canvas.create_rectangle(self.vWorld.canvas_width+min(xVals), self.vWorld.canvas_height-max(yVals), self.vWorld.canvas_width+max(xVals), self.vWorld.canvas_height-min(yVals), fill='', outline='black', width=2)

            #assumes there are some obstacles in the way, finds them
            obs2 = self.find_next_cobs_up(tx, ty, cells)
            
            if line == [-45, -97, -45, -135]:
                print 'obs2:', obs2
            
            if obs2 is None:
                ty = self.vWorld.canvas_height
            else:
                ty = min([obs2[1], obs2[3]])
            obs2 = self.find_next_cobs_down_left(tx, ty, cells)
            
            if line == [-45, -97, -45, -135]:
                print 'obs3:', obs2
            
            if obs2 is None:
                by = -self.vWorld.canvas_height
            else:
                by = max([obs2[1], obs2[3]])
            bx = max([obs[0], obs[2]])
            return [tx, ty, bx, by]
    
    def line_inbetween(self, x1, x2, lines):
        xVals = [x1, x2]
        for line in lines:
            if min(xVals) < line[0] and max(xVals) > line[0]:
                return True
        return False
    
    def contains_cobs_or_cells(self, tx, ty, bx, by, cells):
        mxVals = [tx, bx]
        myVals = [ty, by]
        mpoints = (tx, ty, bx, ty, bx, by, tx, by)
        for rect in self.vWorld.cobs:
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]

            rPoints = (min(xVals), max(yVals), max(xVals), max(yVals), max(xVals), min(yVals), min(xVals), min(yVals))
            
            if self.is_plus_sign([tx, ty, bx, by], rect) == True:
                return True

            i = 0
            while i < 8:
                p1 = [mpoints[i], mpoints[i+1]]
                p2 = [rPoints[i], rPoints[i+1]]
                if (p1[0] < max(xVals) and p1[0] > min(xVals) and p1[1] < max(yVals) and p1[1] > min(yVals)) or ((p2[0] < max(mxVals) and p2[0] > min(mxVals) and p2[1] < max(myVals) and p2[1] > min(myVals))):
                    return True
                i += 2
        for cell in cells:
            xVals = [cell[0], cell[2]]
            yVals = [cell[1], cell[3]]
            rPoints = (min(xVals), max(yVals), max(xVals), max(yVals), max(xVals), min(yVals), min(xVals), min(yVals))
            
            if self.is_plus_sign([tx, ty, bx, by], cell) == True:
                return True

            i = 0
            while i < 8:
                p1 = [mpoints[i], mpoints[i+1]]
                p2 = [rPoints[i], rPoints[i+1]]
                if (p1[0] < max(xVals) and p1[0] > min(xVals) and p1[1] < max(yVals) and p1[1] > min(yVals)) or ((p2[0] < max(mxVals) and p2[0] > min(mxVals) and p2[1] < max(myVals) and p2[1] > min(myVals))):
                    return True
                i += 2
        return False

    def is_plus_sign(self, rect1, rect2):
        xVals1 = [rect1[0], rect1[2]]
        yVals1 = [rect1[1], rect1[3]]

        xVals2 = [rect2[0], rect2[2]]
        yVals2 = [rect2[1], rect2[3]]

        #1 is vertical
        if min(xVals1) >= min(xVals2) and max(xVals1) <= max(xVals2) and min(yVals2) >= min(yVals1) and max(yVals2) <= max(yVals1):
            return True
        if min(xVals2) >= min(xVals1) and max(xVals2) <= max(xVals1) and min(yVals1) >= min(yVals2) and max(yVals1) <= max(yVals2):
            return True
        return False

    def contains_cobs_or_cells_prnt(self, tx, ty, bx, by, cells):
        mxVals = [tx, bx]
        myVals = [ty, by]
        mpoints = (tx, ty, bx, ty, bx, by, tx, by)
        print tx, ty, bx, by
        for rect in self.vWorld.cobs:
            print 'rect', rect
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            rPoints = (min(xVals), max(yVals), max(xVals), max(yVals), max(xVals), min(yVals), min(xVals), min(yVals))
            i = 0
            while i < 8:
                p1 = [mpoints[i], mpoints[i+1]]
                p2 = [rPoints[i], rPoints[i+1]]
                if (p1[0] < max(xVals) and p1[0] > min(xVals) and p1[1] < max(yVals) and p1[1] > min(yVals)) or ((p2[0] < max(mxVals) and p2[0] > min(mxVals) and p2[1] < max(myVals) and p2[1] > min(myVals))):
                    return True
                i += 2

        for cell in cells:
            print 'cell', cell
            xVals = [cell[0], cell[2]]
            yVals = [cell[1], cell[3]]
            rPoints = (min(xVals), max(yVals), max(xVals), max(yVals), max(xVals), min(yVals), min(xVals), min(yVals))
            i = 0
            while i < 8:
                p1 = [mpoints[i], mpoints[i+1]]
                p2 = [rPoints[i], rPoints[i+1]]
                if (p1[0] < max(xVals) and p1[0] > min(xVals) and p1[1] < max(yVals) and p1[1] > min(yVals)) or ((p2[0] < max(mxVals) and p2[0] > min(mxVals) and p2[1] < max(myVals) and p2[1] > min(myVals))):
                    return True
                i += 2
        return False

    def find_next_line_right(self, x, y, lines):
        next = None
        #print 'find_next_line ran'
        for line in lines:
            xVals = [line[0], line[2]]
            yVals = [line[1], line[3]]
            if (next is None and min(xVals) > x and y <= max(yVals) and y >= min(yVals)) or (not next is None and min(xVals) > x and y <= max(yVals) and y >= min(yVals) and abs(min([next[0], next[2]]) - x) > abs(min(xVals) - x)):
                #print 'find next worked'
                next = line
        return next

    #Add detection of free cells
    def find_next_cobs_up(self, x, y, cells):
        next = None
        for rect in self.vWorld.cobs:
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            if (next is None and x >= min(xVals) and x < max(xVals) and y <= min(yVals)) or (not next is None and x >= min(xVals) and x < max(xVals) and y <= min(yVals) and abs(min(yVals) - y) < abs(min([next[1], next[3]]) - y)):
                next = rect
        for cell in cells:
            xVals = [cell[0], cell[2]]
            yVals = [cell[1], cell[3]]
            if (next is None and x >= min(xVals) and x < max(xVals) and y <= min(yVals)) or (not next is None and x >= min(xVals) and x < max(xVals) and y <= min(yVals) and abs(min(yVals) - y) < abs(min([next[1], next[3]]) - y)):
                next = cell
        return next

    def find_next_cobs_up_left(self, x, y, cells):
        next = None
        for rect in self.vWorld.cobs:
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            if (next is None and x > min(xVals) and x <= max(xVals) and y <= min(yVals)) or (not next is None and x > min(xVals) and x <= max(xVals) and y <= min(yVals) and abs(min(yVals) - y) < abs(min([next[1], next[3]]) - y)):
                next = rect
        for cell in cells:
            xVals = [cell[0], cell[2]]
            yVals = [cell[1], cell[3]]
            if (next is None and x > min(xVals) and x <= max(xVals) and y <= min(yVals)) or (not next is None and x > min(xVals) and x <= max(xVals) and y <= min(yVals) and abs(min(yVals) - y) < abs(min([next[1], next[3]]) - y)):
                next = cell
        return next

    #Add detection of free cells
    def find_next_cobs_down(self, x, y, cells):
        next = None
        for rect in self.vWorld.cobs:
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            if (next is None and x >= min(xVals) and x < max(xVals) and y >= max(yVals)) or (not next is None and x >= min(xVals) and x < max(xVals) and y >= max(yVals) and abs(max(yVals) - y) < abs(max([next[1], next[3]]) - y)):
                next = rect
        for cell in cells:
            xVals = [cell[0], cell[2]]
            yVals = [cell[1], cell[3]]
            if (next is None and x >= min(xVals) and x < max(xVals) and y >= max(yVals)) or (not next is None and x >= min(xVals) and x < max(xVals) and y >= min(yVals) and abs(min(yVals) - y) < abs(min([next[1], next[3]]) - y)):
                next = cell
        return next

    def find_next_cobs_down_left(self, x, y, cells):
        next = None
        for rect in self.vWorld.cobs:
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            if (next is None and x > min(xVals) and x <= max(xVals) and y >= max(yVals)) or (not next is None and x > min(xVals) and x <= max(xVals) and y >= max(yVals) and abs(max(yVals) - y) <= abs(max([next[1], next[3]]) - y)):
                next = rect
        for cell in cells:
            xVals = [cell[0], cell[2]]
            yVals = [cell[1], cell[3]]
            if (next is None and x > min(xVals) and x <= max(xVals) and y >= max(yVals)) or (not next is None and x > min(xVals) and x <= max(xVals) and y >= min(yVals) and abs(min(yVals) - y) <= abs(min([next[1], next[3]]) - y)):
                next = cell
        return next

    #Add detection of free cells
    def find_next_cobs(self, x, y, cells, lines):
        next = None
        for rect in self.vWorld.cobs:
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            if (next is None  and y <= max(yVals) and y >= min(yVals) and x < min(xVals)) or (not next is None and y <= max(yVals) and y >= min(yVals) and x < min(xVals) and abs(min([next[0], next[2]])-x) > abs(min(xVals)-x)):
                next = rect
        for cell in cells:
            xVals = [cell[0], cell[2]]
            yVals = [cell[1], cell[3]]
            if (next is None  and y <= max(yVals) and y >= min(yVals) and x < min(xVals)) or (not next is None and y <= max(yVals) and y >= min(yVals) and x < min(xVals) and abs(min([next[0], next[2]])-x) > abs(min(xVals)-x)):
                next = cell
        for line in lines:
            xVals = [line[0], line[2]]
            yVals = [line[1], line[3]]
            if (next is None  and y <= max(yVals) and y >= min(yVals) and x < min(xVals)) or (not next is None and y <= max(yVals) and y >= min(yVals) and x < min(xVals) and abs(min([next[0], next[2]])-x) > abs(min(xVals)-x)):
                next = line
        return next

    def find_next_cobs_left(self, x, y, cells, lines):
        next = None
        for rect in self.vWorld.cobs:
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            if (next is None  and y <= max(yVals) and y >= min(yVals) and x > max(xVals)) or (not next is None and y <= max(yVals) and y >= min(yVals) and x > max(xVals) and abs(max([next[0], next[2]])-x) > abs(max(xVals)-x)):
                next = rect
        for cell in cells:
            xVals = [cell[0], cell[2]]
            yVals = [cell[1], cell[3]]
            if (next is None  and y <= max(yVals) and y >= min(yVals) and x > max(xVals)) or (not next is None and y <= max(yVals) and y >= min(yVals) and x > max(xVals) and abs(max([next[0], next[2]])-x) > abs(max(xVals)-x)):
                next = cell
        for line in lines:
            xVals = [line[0], line[2]]
            yVals = [line[1], line[3]]
            if (next is None  and y <= max(yVals) and y >= min(yVals) and x > max(xVals)) or (not next is None and y <= max(yVals) and y >= min(yVals) and x > max(xVals) and abs(max([next[0], next[2]])-x) > abs(max(xVals)-x)):
                next = line
        return next

    def inside_rect_down(self, x, y):
        for rect in self.vWorld.cobs:
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            if y == max(yVals) and x > min(xVals) and x <= max(xVals):
                return True
        return False

    def inside_rect_down_prnt(self, x, y):
        for rect in self.vWorld.cobs:
            print rect
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            if y == max(yVals) and x > min(xVals) and x <= max(xVals):
                #print 'returned true'
                return True
        #print 'returned false'
        return False

    def inside_rect_up(self, x, y):
        for rect in self.vWorld.cobs:
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            if y == min(yVals) and x >= min(xVals) and x <= max(xVals):
                return True
        return False


    def inside_rect_up_prnt(self, x, y, c_obs_list):
        for rect in c_obs_list:
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            print min(yVals)
            if y == min(yVals):
                #print 'sameY', y
                #print x, rect
                if x >= min(xVals) and x <= max(xVals):
                    return True
        return False

    def contained(self, tx, ty, bx, by, cells):
        for rect in cells:
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            if min([tx, bx]) >= min(xVals) and max([tx, bx]) <= max(xVals) and min([ty, by]) >= min(yVals) and max([ty, by]) <= max(yVals):
                return True
        return False

    def inside_cell(self, x, y, cells):
        for rect in cells:
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            if x >= min(xVals) and x < max(xVals) and y >= min(yVals) and y <= max(yVals):
                return True
        return False

    #finds a line with the same x, and next lower y
    def find_next_line(self, x, y, lines):
        closest = None
        for line in lines:
            xVals = [line[0], line[2]]
            yVals = [line[1], line[3]]
            if (closest is None and line[0] == x and max(yVals) < y) or ((not closest is None) and line[0] == x and max(yVals) > max([closest[1], closest[3]]) and max(yVals) < y):
                closest = line
        return closest

    def line_intersect_cobs(self, x, y, x2, y2, cX, cY, isHorizontal):
        lVal = y
        gVal = y2
        for rect in self.vWorld.cobs:
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            if cX > min(xVals) and cX < max(xVals) and cY > min(yVals) and cY < max(yVals):
                return True
            if cX > min(xVals) and cX < max(xVals) and (cY == min(yVals) or cY == max(yVals)):
                return True
            if isHorizontal:
                if y > rect[1] and y < rect[3]:
                    if x < rect[0]:
                        return rect[0]
                    else:
                        return rect[2]
            else:
                if x > min(xVals) and x < max(xVals):
                    if cY > max(yVals):
                        if lVal is None or (abs(cY - max(yVals)) < abs(lVal - max(yVals))):
                            lVal = max(yVals)
                    elif cY < min(yVals):
                        if gVal is None or (abs(cY - min(yVals)) < abs(gVal - min(yVals))):
                            gVal = min(yVals)
        if lVal is None or gVal is None:
            return False
        else:
            return [lVal, gVal]


    def line_intersect_cobs_prnt(self, x, y, x2, y2, cX, cY, isHorizontal):
        lVal = None
        gVal = None
        for rect in self.vWorld.cobs:
            xVals = [rect[0], rect[2]]
            yVals = [rect[1], rect[3]]
            if cX > min(xVals) and cX < max(xVals) and cY > min(yVals) and cY < max(yVals):
                return True
            if cX > min(xVals) and cX < max(xVals) and (cY == min(yVals) or cY == max(yVals)):
                return True
            if isHorizontal:
                if y > rect[1] and y < rect[3]:
                    if x < rect[0]:
                        return rect[0]
                    else:
                        return rect[2]
            else:
                if x >= min(xVals) and x <= max(xVals):
                    #print 'found obs'
                    if cY > max(yVals):
                        #print 'cy > max'
                        if lVal is None or (abs(cY - max(yVals)) < abs(cY - lVal)):
                            if cY == y:
                                #print 'changed lVal'
                                lVal = max(yVals)
                    elif cY < min(yVals):
                        #print 'cy < min'
                        if gVal is None or (abs(cY - min(yVals)) < abs(cY - gVal)):
                            if cY == y2:
                                #print 'changed gVal'
                                gVal = min(yVals)
        if lVal is None and gVal is None:
            return False
        else:
            return [gVal, lVal]

    def two_cells_connected(self, cell1, cell2):
        # Given two free cells, cell1 and cell2.
        # return connecting point[x,y] if two cells are connected
        # return False if not connected
        xVals1 = [cell1[0], cell1[2]]
        yVals1 = [cell1[1], cell1[3]]

        xVals2 = [cell2[0], cell2[2]]
        yVals2 = [cell2[1], cell2[3]]

        orderX = sorted([xVals1[0], xVals1[1], xVals2[0], xVals2[1]])
        orderY = sorted([yVals1[0], yVals1[1], yVals2[0], yVals2[1]])

        duplicateX = [False, None]
        for xVal1 in xVals1:
            for xVal2 in xVals2:
                if xVal1 == xVal2:
                    if len(duplicateX) == 2:
                        duplicateX = [True, [xVal1, xVal2]]
                    else:
                        duplicateX.append([xVal1, xVal2])

        duplicateY = [False, None]
        for yVal1 in yVals1:
            for yVal2 in yVals2:
                if yVal1 == yVal2:
                    if len(duplicateY) == 2:
                        duplicateY = [True, [yVal1, yVal2]]
                    else:
                        duplicateY.append([yVal1, yVal2])
        '''
        if duplicateX[0] == True and duplicateY[0] == True and (len(duplicateX) < 3 and len(duplicateY) < 3):
            print 'dupe 1'
            #Top left
            if min(xVals1) == duplicateX[1][0] and max(yVals1) == duplicateY[1][0]:
                return False
            #Bottom left
            elif min(xVals1) == duplicateX[1][0] and min(yVals1) == duplicateY[1][0]:
                return False
            #Top right
            elif max(xVals1) == duplicateX[1][0] and max(yVals1) == duplicateY[1][0]:
                return False
            #Bottom right
            elif max(xVals1) == duplicateX[1][0] and min(yVals1) == duplicateY[1][0]:
                return False
            return True
        elif duplicateX[0] == True and duplicateY[0] == True and (len(duplicateX) > 2 or len(duplicateY) > 2):
            print 'dupe 2'
            return True
        '''
        if duplicateX[0] == True:
            #print 'dupe 3'
            #Yvals overlap
            if (min(yVals1) <= max(yVals2) and min(yVals1) >= min(yVals2)) or (min(yVals2) <= max(yVals1) and min(yVals2) >= min(yVals1)):
                return True
            return False
        elif duplicateY[0] == True:
            #print 'dupe 4'
            #Xvals overlap
            if (min(xVals1) <= max(xVals2) and min(xVals1) >= min(xVals2)) or (min(xVals2) <= max(xVals1) and min(xVals2) >= min(xVals1)):
                return True
            return False
        else:
            #print 'end'
            return False

    def compute_free_points(self, f_cell_list):
        # Obstacle free cells are given in f_cell_list
        # This function returns a list of points, each point is on overlapping edge of two connected obstacle free cells.
        points = []
        for cell1 in f_cell_list:
            for cell2 in f_cell_list:
                if not cell1 == cell2 and self.two_cells_connected(cell1, cell2):
                    #print 'TWO CELLS CONNECTED'
                    #Calculate point
                    xVals1 = [cell1[0], cell1[2]]
                    yVals1 = [cell1[1], cell1[3]]

                    xVals2 = [cell2[0], cell2[2]]
                    yVals2 = [cell2[1], cell2[3]]

                    side = self.find_overlap_side(cell1, cell2)

                    point = None

                    if side == 0:
                        x = min(xVals1)

                        ty = max(yVals1)
                        if max(yVals2) < ty:
                            ty = max(yVals2)

                        by = min(yVals1)
                        if min(yVals2) > by:
                            by = min(yVals2)

                        y = (ty + by)/2

                        point = [x, y]

                    elif side == 1:
                        y = max(yVals1)

                        tx = max(xVals1)
                        if max(xVals2) < tx:
                            tx = max(xVals2)

                        bx = min(xVals1)
                        if min(xVals2) > bx:
                            bx = min(xVals2)

                        x = (tx + bx)/2

                        point = [x, y]

                    elif side == 2:
                        x = max(xVals1)

                        ty = max(yVals1)
                        if max(yVals2) < ty:
                            ty = max(yVals2)

                        by = min(yVals1)
                        if min(yVals2) > by:
                            by = min(yVals2)

                        y = (ty + by)/2

                        point = [x, y]

                    elif side == 3:
                        y = min(yVals1)

                        tx = max(xVals1)
                        if max(xVals2) < tx:
                            tx = max(xVals2)

                        bx = min(xVals1)
                        if min(xVals2) > bx:
                            bx = min(xVals2)

                        x = (tx + bx)/2

                        point = [x, y]

                    if not point is None and not point in points:
                        point.append(cell1)
                        point.append(cell2)
                        points.append(point)
        print len(points)
        '''
        num = 0
        for i in range(num, num+1):
        #for i in range(0, len(points)):
            print points[i]
            point = points[i]
            cell = point[2]
            self.vWorld.canvas.create_rectangle(self.vWorld.canvas_width+cell[0], self.vWorld.canvas_height-cell[1], self.vWorld.canvas_width+cell[2], self.vWorld.canvas_height-cell[3], fill='', outline='red')
            cell = point[3]
            #print self.two_cells_connected_prnt(point[2], point[3])
            self.vWorld.canvas.create_rectangle(self.vWorld.canvas_width+cell[0], self.vWorld.canvas_height-cell[1], self.vWorld.canvas_width+cell[2], self.vWorld.canvas_height-cell[3], fill='', outline='red')
            self.vWorld.canvas.create_oval(self.vWorld.canvas_width+points[i][0]-4, self.vWorld.canvas_height-points[i][1]-4, self.vWorld.canvas_width+points[i][0]+4, self.vWorld.canvas_height-points[i][1]+4, outline = "red")
        '''
        return points

    #0 if left side of cell1, 1 if top side of cell 1, etc
    def find_overlap_side(self, cell1, cell2):
        xVals1 = [cell1[0], cell1[2]]
        yVals1 = [cell1[1], cell1[3]]

        xVals2 = [cell2[0], cell2[2]]
        yVals2 = [cell2[1], cell2[3]]

        if min(xVals1) in xVals2:
            return 0
        elif max(xVals1) in xVals2:
            return 2
        elif max(yVals1) in yVals2:
            return 1
        elif(min(yVals1) in yVals2):
            return 3 

    def two_cells_connected_prnt(self, cell1, cell2):
        # Given two free cells, cell1 and cell2.
        # return connecting point[x,y] if two cells are connected
        # return False if not connected
        xVals1 = [cell1[0], cell1[2]]
        yVals1 = [cell1[1], cell1[3]]

        xVals2 = [cell2[0], cell2[2]]
        yVals2 = [cell2[1], cell2[3]]

        orderX = sorted([xVals1[0], xVals1[1], xVals2[0], xVals2[1]])
        orderY = sorted([yVals1[0], yVals1[1], yVals2[0], yVals2[1]])

        duplicateX = [False, None]
        for xVal1 in xVals1:
            for xVal2 in xVals2:
                if xVal1 == xVal2:
                    #print 'duplicateX'
                    duplicateX = [True, [xVal1, xVal2]]
        duplicateY = [False, None]
        for yVal1 in yVals1:
            for yVal2 in yVals2:
                if yVal1 == yVal2:
                    #print 'duplicateY'
                    duplicateY = [True, [yVal1, yVal2]]
        
        if duplicateX[0] == True and duplicateY[0] == True:
            #print 'both duplicate'
            #Top left
            if min(xVals1) == duplicateX[1][0] and max(yVals1) == duplicateY[1][0]:
                return False
            #Bottom left
            elif min(xVals1) == duplicateX[1][0] and min(yVals1) == duplicateY[1][0]:
                return False
            #Top right
            elif max(xVals1) == duplicateX[1][0] and max(yVals1) == duplicateY[1][0]:
                return False
            #Bottom right
            elif max(xVals1) == duplicateX[1][0] and min(yVals1) == duplicateY[1][0]:
                return False
            return True
        elif duplicateX[0] == True:
            #print 'only duplicateX'
            #Yvals overlap
            if (min(yVals1) <= max(yVals2) and min(yVals1) >= min(yVals2)) or (min(yVals2) <= max(yVals1) and min(yVals2) >= min(yVals1)):
                return True
            return False
        elif duplicateY[0] == True:
            #print 'only duplicateY'
            if (min(xVals1) <= max(xVals2) and min(xVals1) >= min(xVals2)) or (min(xVals2) <= max(xVals1) and min(xVals2) >= min(xVals1)):
                return True
            return False
        else:
            return False

    def connected(self, point1, point2, cell_list):
        # given two points in c-space and list of free cells.
        # return True if point1 and point2 are connected by a free cell
        # otherwise return False
        for cell in cell_list:
            xVals = [cell[0], cell[2]]
            yVals = [cell[1], cell[3]]
            #point 1 inside
            if point1[0] >= min(xVals) and point1[0] <= max(xVals) and point1[1] >= min(yVals) and point1[1] <= max(yVals):
                #point 2 inside
                if point2[0] >= min(xVals) and point2[0] <= max(xVals) and point2[1] >= min(yVals) and point2[1] <= max(yVals):
                    #check if both on same edge
                    if self.on_wall(point1, cell) == self.on_wall(point2, cell):
                        pass
                    else:
                        return True
        return False

    #0 = left wall, 1 = top wall, etc
    def on_wall(self, point, cell):
        xVals = [cell[0], cell[2]]
        yVals = [cell[1], cell[3]]

        if point[0] == min(xVals):
            return 0
        elif point[0] == max(xVals):
            return 2
        elif point[1] == min(yVals):
            return 4
        elif point[1] == max(yVals):
            return 1

class GUI(object):
    def __init__(self, gui_root, vWorld, endCommand):
        self.gui_root = gui_root
        gui_root.title("Motion Planner")
        self.endCommand = endCommand
        self.vWorld = vWorld
        self.start = [200, 0] # robot's start location, goal location is user defined
        self.initUI()
        return

    def initUI(self):
        #creating tje virtual appearance of the robot
        canvas_width = 440 # half width
        canvas_height = 300 # half height
        self.vWorld.canvas_width = canvas_width
        self.vWorld.canvas_height = canvas_height
        rCanvas  = tk.Canvas(self.gui_root, bg="light gray", width=canvas_width*2, height= canvas_height*2)
        self.vWorld.canvas = rCanvas
        rCanvas.pack()

        button0 = tk.Button(self.gui_root,text="Grid")
        button0.pack(side='left')
        button0.bind('<Button-1>', self.drawGrid)

        button1 = tk.Button(self.gui_root,text="Clear")
        button1.pack(side='left')
        button1.bind('<Button-1>', self.clearCanvas)

        button2 = tk.Button(self.gui_root,text="Map")
        button2.pack(side='left')
        button2.bind('<Button-1>', self.drawMap)

        button9 = tk.Button(self.gui_root,text="Exit")
        button9.pack(side='left')
        button9.bind('<Button-1>', self.endCommand)

        rCanvas.bind("<Button-1>", self.getGoal)
        return

    def drawGrid(self, event=None):
        print "draw Grid"
        canvas_width = self.vWorld.canvas_width
        canvas_height = self.vWorld.canvas_height
        rCanvas = self.vWorld.canvas
        x1 = 0
        x2 = canvas_width*2
        y1 = 0
        y2 = canvas_height*2
        del_x = 20
        del_y = 20
        num_x = x2 / del_x
        num_y = y2 / del_y
        # draw center (0,0)
        rCanvas.create_rectangle(canvas_width-3,canvas_height-3,canvas_width+3,canvas_height+3, fill="red")
        # horizontal grid
        for i in range (0,num_y):
            y = i * del_y
            rCanvas.create_line(x1, y, x2, y, fill="yellow")
        # verticle grid
        for j in range (0, num_x):
            x = j * del_x
            rCanvas.create_line(x, y1, x, y2, fill="yellow")
        return

    def drawMap(self, event=None):
        self.vWorld.draw_map()

    def clearCanvas(self, event=None):
        rCanvas = self.vWorld.canvas
        rCanvas.delete("all")
        return

    def getGoal(self, event):
        self.vWorld.canvas.create_oval(event.x-4, event.y-4, event.x+4, event.y+4, outline = "blue")

        canvas_width = self.vWorld.canvas_width
        canvas_height = self.vWorld.canvas_height
        self.vWorld.goal_x = event.x - canvas_width
        self.vWorld.goal_y = canvas_height - event.y 
        print "selected goal: ",self.vWorld.goal_x, self.vWorld.goal_y
        s_point = self.start
        g_point = [self.vWorld.goal_x, self.vWorld.goal_y]
        print 'start pose(%s, %s): ' % (s_point[0], s_point[1])
        print 'goal pose(%s, %s): ' % (g_point[0], g_point[1]) 
        mp = MotionPlanner(self.vWorld, s_point, g_point)
        mp.worker()
        return

class VirtualWorld(object):
    def __init__(self, gui_root):
        self.gui_root = gui_root
        self.gui_handle = None
        self.vWorld = None
        self.create_world()
        return

    def create_world(self):
        self.vWorld = virtual_world()      
        #objects in the world
        self.vWorld.map = []

        #project 3-1
        #rect1 = [-50, 80, 50, 120]
        #rect2 = [100, -50, 140, 50]
        #rect3 = [-160, -50, -120, 50]
        #rect4 = [-50,-180, 50, -140]

        #project 3-2
        #rect1 = [-20, 80, 20, 120]
        #rect2 = [100, -20, 140, 20]
        #rect3 = [-20, -120, 20, -80]
        #rect4 = [-260,-30, -220, 30]
        #rect5 = [-220, -70, -180, -30]
        #rect6 = [-220, 30, -180, 70]

        '''
        
        #bounder of board
        rect1 = [-100, -180, 0, -140]
        rect2 = [-140, -180, -100, -80]
        rect3 = [-100, 140, 0, 180]
        rect4 = [-140, 80, -100, 180]
        rect5 = [0, -50, 40, 50]
        rect6 = [-260, -20, -220, 20]
        rect7 = [40, 60, 140, 100]

        self.vWorld.add_obstacle(rect1)
        self.vWorld.add_obstacle(rect2)
        self.vWorld.add_obstacle(rect3)
        self.vWorld.add_obstacle(rect4)
        self.vWorld.add_obstacle(rect5)
        self.vWorld.add_obstacle(rect6)
        self.vWorld.add_obstacle(rect7)
        
        
        obs = [
        [-250, 158, -230, 178],
        [265, 34, 285, 54],
        [-282, 107, -262, 127],
        [-244, -131, -224, -111],
        [106, 43, 126, 63],
        [-129, 30, -109, 50],
        [-200, -184, -180, -164],
        [-296, -49, -276, -29],
        [-96, 148, -76, 168],
        [247, 122, 267, 142],
        [181, 61, 201, 81],
        [-24, -68, -4, -48],
        [165, 58, 185, 78],
        [228, -171, 248, -151],
        [-45, -10, -25, 10],
        [-110, -114, -90, -94],
        [-74, 87, -54, 107],
        [138, -184, 158, -164],
        [-287, -105, -267, -85],
        [-148, -159, -128, -139]
        ]
        '''
        '''
        obs2 = [
        [-82, 33, -62, 53],
        [214, 108, 234, 128],
        [-2, 46, 18, 66],
        [-68, 157, -48, 177],
        [-47, 20, -27, 40],
        [-85, -6, -65, 14],
        [-159, -185, -139, -165],
        [-169, 158, -149, 178],
        [-234, 140, -214, 160],
        [-258, 52, -238, 72],
        [-236, -128, -216, -108],
        [66, -111, 86, -91],
        [-184, -157, -164, -137],
        [-93, -183, -73, -163],
        [-12, 97, 8, 117],
        [-80, -69, -60, -49],
        [-133, -138, -113, -118],
        [-106, 34, -86, 54],
        [-27, 7, -7, 27],
        [125, 112, 145, 132]
        ]

        for ob in obs2:
            self.vWorld.add_obstacle(ob)
        '''
        # robot's work space boundary
        canvas_width = 300
        canvas_height = 200
        self.vWorld.area = [-300,-200,300,200]
        
        i = 0
        while i < 6:
            x = random.randint(-canvas_width, canvas_width)
            y = random.randint(-canvas_height, canvas_height)
            #print x, y
            if x + 20 < canvas_width and y - 20 > -canvas_height:
                #print 'added'
                self.vWorld.add_obstacle([x, y-20, x+20, y])
                i += 1
                print [x, y-20, x+20, y]
        
        self.gui_handle = GUI(self.gui_root, self.vWorld, self.stopProg)
        return

    def stopProg(self, event=None):
        self.gui_root.quit()
        return

def main():
    m = tk.Tk() #root
    v_world = VirtualWorld(m)
    m.mainloop()
    return

if __name__== "__main__":
    sys.exit(main())
