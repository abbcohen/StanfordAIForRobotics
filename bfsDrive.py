'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.
   This is a program that is provided to students in Robot AI class.
   Students use this it to build different Hamster behaviors.

   Name:          tk_behaviors_starter.py
   By:            Qin Chen
   Last Updated:  5/10/18

   PROPRIETARY and CONFIDENTIAL
   ========================================================================*/

   left is 0 right is 1
'''
import sys
import time
import threading
import Tkinter as tk
from HamsterAPI.comm_ble import RobotComm	# no dongle
import starter_grid_graph_display as display
from starter_grid_graph import *
from starter_bfs import *

################################
# Hamster Control
################################
class RobotBehaviorThread(threading.Thread):
	def __init__(self, robotList, DriveCommands, gridgraph):
		super(RobotBehaviorThread, self).__init__()
		self.go = False
		self.done = False
		self.robotList = robotList
		self.commands = DriveCommands
		self.gridgraph = gridgraph
		self.step=0
		return

	def run(self):
		print self.robotList
		robot=None
		while not self.done:
			for robot in self.robotList:
				if (self.step==0):
						self.driveForward(robot)
				elif(robot.get_floor(0)<50 and robot.get_floor(1)<50): #intersection
					if (self.step==len(self.commands)):
				 		self.done=True
				 		break
					print"in"
					# print("from: " + str(self.commands[self.step-1]) + " to: " + str(self.commands[self.step]))
					if(self.commands[self.step-1] == "left"):
						if(self.commands[self.step] == "left"):
							self.driveForward(robot)
						elif(self.commands[self.step] == "up"):
							self.driveRight(robot)
						elif(self.commands[self.step] == "down"):
							self.driveLeft(robot)
					elif(self.commands[self.step-1] == "right"):
						if(self.commands[self.step] == "right"):
							self.driveForward(robot)
						elif(self.commands[self.step] == "up"):
							self.driveLeft(robot)
						elif(self.commands[self.step]== "down"):
							self.driveRight(robot)
					elif(self.commands[self.step-1] =="up"):
						if(self.commands[self.step] == "left"):
							self.driveLeft(robot)
						elif(self.commands[self.step] == "right"):
							self.driveRight(robot)
						elif(self.commands[self.step] == "up"):
							self.driveForward(robot)
					elif(self.commands[self.step-1] == "down"):
						if(self.commands[self.step] == "left"):
							self.driveRight(robot)
						elif(self.commands[self.step] =="right"):
							self.driveLeft(robot)
						elif(self.commands[self.step] == "down"):
							self.driveForward(robot)
				#FOLLOW THE LINE:
				else:
					# print"driving"
					if(robot.get_floor(0) < 50):
						robot.set_wheel(0, 0)
					else: 
						robot.set_wheel(0, 20)

					if(robot.get_floor(1) < 50):
						robot.set_wheel(1, 0)
					else:
						robot.set_wheel(1, 20)
		robot.set_wheel(1, 0)
		robot.set_wheel(1, 0)
		print("done")
					
		if robot:
			robot.reset()
			time.sleep(0.1)
		return

	def driveForward(self, robot):
		print"driving Forward"
		self.stepOffLine()
		self.step=self.step+1
	def driveRight(self,robot):
		print"turning right"
		robot.set_wheel(0, 20)
		robot.set_wheel(1, - 20)
		self.stepOffLine()
		self.step=self.step+1
	def driveLeft(self, robot):
		print"turning left"
		robot.set_wheel(0, -20)
		robot.set_wheel(1, 20)
		self.stepOffLine()
		self.step=self.step+1
	def stepOffLine(self):
		for robot in self.robotList:
			if(robot.get_proximity(0)>50 and robot.get_proximity(1)>50):
				print "welp... shit"
				robot.set_led(0,4)
				robot.set_led(1,4)
				robot.set_wheel(0, 0)
				robot.set_wheel(1, 0)
				# self.recalibrate()
			print"we lit"
			time.sleep(1.2)
			robot.set_wheel(0, 20)
			robot.set_wheel(1, 20)
			time.sleep(1)
	def Recalibrate(self):
		graph.set_start(self.ShortestPath[self.step])

		# self.newStart=self.ShortestPath[self.step]
		# bfs = BFS(self.graph.nodes)
  #       allNewPaths=bfs.bfs_paths(self.newStart, self.goal_node)
  #       newShortestPath=bfs.shortest(allNewPaths)
  #       self.highlight_path(newShortestPath)
  #       self.gridgraph.DriveCommands=bfs.driveLogic(newShortestPath)
  #       self.gridgraph.ShortestPath=shortestPath

	class GUI(object):
		def __init__(self, root, robot_control):
			self.root = root
			self.robot_control = robot_control
			root.geometry('250x30')
			root.title('Hamster Control')

			b1 = tk.Button(root, text='Go')
			b1.pack(side='left')
			b1.bind('<Button-1>', self.startProg)

			b2 = tk.Button(root, text='Stop')
			b2.pack(side='left')
			b2.bind('<Button-2>', self.stopProg)
			return
		
		def startProg(self, event=None):
			self.robot_control.go = True
			return

		def stopProg(self, event=None):
			self.robot_control.done = True		
			self.root.quit() 	# close window
			return

	#################################
	# Don't change any code below!! #
	#################################

	# def main():
	#     # instantiate COMM object
	#     gMaxRobotNum = 1; # max number of robots to control
	#     comm = RobotComm(gMaxRobotNum)
	#     comm.start()
	#     print 'Bluetooth starts'  
	#     robotList = comm.robotList

	#     behaviors = RobotBehaviorThread(robotList)
	#     behaviors.start()

	#     frame = tk.Tk()
	#     GUI(frame, behaviors)
	#     frame.mainloop()

	#     comm.stop()
	#     comm.join()
	#     print("terminated!")

	# if __name__ == "__main__":
	#     sys.exit(main())