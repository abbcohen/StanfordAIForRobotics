'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.
   Stater program of 3-state obstacle avoidance using FSM.

   Name:          starter_tk_3state_avoid.py
   By:            Qin Chen
   Last Updated:  6/10/18

   PROPRIETARY and CONFIDENTIAL
   ========================================================================*/
'''
import sys
import time
import threading
import logging
import Tkinter as tk
import Queue
from HamsterAPI.comm_ble import RobotComm	# no dongle
#from HamsterAPI.comm_usb import RobotComm	# yes dongle

class Event(object):
    def __init__(self, event_type, event_data):
        self.type = event_type #string
        self.data = event_data #list of number or character depending on type

###############################
# Finite state machine engine #
###############################
class StateMachine(object):
	def __init__(self, name, eventQ_handle):
		self.name = name		# machine name
		self.states = []	# list of lists, [[state name, event, transition, next_state],...]
		self.start_state = None
		self.end_states = []	# list of name strings
		self.q = eventQ_handle
		return

	def set_start_state(self, state_name):
		self.start_state = state_name
		return

	def get_start_state(self):
		return self.start_state
		
	def add_end_state(self, state_name):
		self.end_states.append(state_name)
		return
			
	def add_state(self, state, event, callback, next_state):
		self.states.append([state, event, callback, next_state]) # append to list
		return
	
	# you must set start state before calling run()
	def run(self):
		current_state = self.start_state
		#while not self.q.empty(): # for a machine that has end states
		while True:
			if current_state in self.end_states:
				break
			if not self.q.empty():
				e = self.q.get()
				logging.debug(e)
				for c in self.states:
					if c[0] == current_state and c[1] == e.type:
						c[2]()	# invoke callback function
						if not current_state == c[3]:
							logging.debug("going from " + current_state + " to " + c[3])
						current_state = c[3] 	# next state
						break	# get out of inner for-loop
		return

################################
# Hamster control
################################
class RobotBehavior(object):
	def __init__(self, robot_list):
		self.done = False	# set by GUI button
		self.go = False		# set by GUI button
		self.robot_list = robot_list
		self.robot = None
		self.q = Queue.Queue()	# event queue for FSM
		self.spawn_threads() # thread handles
		self.t_event_watcher = None
		self.score = 0
		return

	def spawn_threads(self):
		###########################################################
		# Two threads are created here.
		# 1. create a watcher thread that reads sensors and registers events: obstacle on left, right or no obstacle. This
		# 	thread runs the method event_watcher() you are going to implement below.
		# 2. Instantiate StateMachine and populate it with avoidance states, triggers, etc. Set start state.
		# 3. Create a thread to run FSM engine.
		###########################################################	
		# 1. create a watcher thread 
		t_event_watcher = threading.Thread(name='watcher thread', target=self.event_watcher, args=(self.q, ))
		t_event_watcher.daemon = True
		t_event_watcher.start()
		self.t_event_watcher = t_event_watcher

		# 2. Instantiate StateMachine and populate it with avoidance states, triggers, etc. Set start state.
		#(self, state, event, callback, next_state)

		#create an instance of finite state machine:
		sm=StateMachine("FSM", self.q)

		#set start state as moving forward
		sm.set_start_state('S_moving_forward')

		#add all the states:
		sm.add_state('S_moving_forward', 'object_left', self.turning_left, 'S_turning_left')
		sm.add_state('S_moving_forward', 'object_right', self.turning_right, 'S_turning_right')
		sm.add_state('S_turning_left', 'object_right', self.turning_right, 'S_turning_right')
		sm.add_state('S_turning_left', 'object_left', self.turning_left, 'S_turning_left')
		sm.add_state('S_turning_right', 'object_right', self.turning_right, 'S_turning_right')
		sm.add_state('S_turning_right', 'object_left', self.turning_left, 'S_turning_left')
		sm.add_state('S_turning_left', 'no_object', self.moving_forward, 'S_moving_forward')
		sm.add_state('S_turning_right', 'no_object', self.moving_forward, 'S_moving_forward')
		sm.add_state('S_moving_forward', 'no_object', self.moving_forward, 'S_moving_forward')


		# 3. Create a thread to run FSM engine.
		t_FSM = threading.Thread(name='FSM', target=sm.run)
		t_FSM.daemon = True
		t_FSM.start()

	def event_watcher(self, q):
		while not self.done:
			if self.robot_list and self.go:
				self.robot = self.robot_list[0]
				
				###########################################################
				# Implement event producer here. The events are obstacle on left, right or no obstacle. Design your
				# logic for what event gets created based on sensor readings.
				###########################################################

				prox_l = self.robot.get_proximity(0)
				prox_r = self.robot.get_proximity(1)
				line_l = self.robot.get_floor(0)
				line_r = self.robot.get_floor(1)
				#print(prox_l,prox_r)
				if(self.robot.get_floor(0)<50 or self.robot.get_floor(1)<50): #crossed black
					self.robot.set_led(0, 1) #turn on left LED
					self.robot.set_led(1, 1) #turn on right LED
					self.robot.set_wheel(1,-40)	#set right wheel backward so it can see the block
					self.robot.set_wheel(0,-40) #set left wheel backward so it can see the block
					time.sleep(1) #sleep to drive back for one second
					self.robot.set_wheel(1,0)	#stop driving back
					self.robot.set_wheel(0,0)
					prox_l = self.robot.get_proximity(0)
					prox_r = self.robot.get_proximity(1)
					if (prox_l > 20 or prox_r > 20): #if either scanner sees an object
						self.score=self.score+1  #add one to the score
						self.robot.set_musical_note(40)
						self.robot.set_led(0, 2) #turn on left LED
						self.robot.set_led(1, 2) #turn on right LED
						print(self.score)
					else: 
						self.robot.set_led(0, 4) #turn on left LED
						self.robot.set_led(1, 4) #turn on right LED
						print("nah")
						print(prox_l,prox_r)
					if(self.score==3):
						self.robot.set_wheel(1,60) #spin
						self.robot.set_wheel(0,-60) 
						self.robot.set_musical_note(40)
						time.sleep(2)
						self.stopProg
					self.robot.set_wheel(1,20) #spin
					self.robot.set_wheel(0,-20) 
					self.robot.set_led(0, 0)
					self.robot.set_led(1, 0)
					time.sleep(1.5)
					self.robot.set_musical_note(0)
					self.robot.set_wheel(1,30) #drive off the line
					self.robot.set_wheel(0,30)
					time.sleep(2)
				elif (prox_l < 20 or prox_r < 20):
					alert_event = Event("no_object", [prox_l,prox_r])
					q.put(alert_event)
					#print('no object')
				elif(abs(prox_r-prox_l)<5):
					alert_event = Event("no_object", [prox_l,prox_r])
					q.put(alert_event)
				elif((prox_l > 20 or prox_r >20) and (prox_l == 0 or prox_r==0)):
					self.robot.set_wheel(1,-30)
					self.robot.set_wheel(0,-30) 
					time.sleep(1.5)
					alert_event = Event("no_object", [prox_l,prox_r])
					q.put(alert_event)
				elif (prox_l > prox_r): #obstacle on the left
					alert_event = Event("object_left", [prox_l,prox_r])
					q.put(alert_event)
					print('object left')
				elif (prox_r > prox_l): #obstacle on the left
					alert_event = Event("object_right", [prox_l,prox_r])
					q.put(alert_event)
					print("object right")
			time.sleep(0.1)
		return

	#######################################
	# Implement Hamster movements to avoid obstacle
	#######################################
	def turning_left(self):
		if not self.done:
			for robot in self.robot_list:
				print("left")
				robot.set_wheel(1,10)
				robot.set_wheel(0,-10) 

	def turning_right(self):
		if not self.done:
			for robot in self.robot_list:
				print("right")
				robot.set_wheel(0,10)
				robot.set_wheel(1,-10) 

	def moving_forward(self):
		if not self.done:
			for robot in self.robot_list:
				#print("forward")
				robot.set_wheel(0,50)
				robot.set_wheel(1,50) 
		  
class GUI(object):
	def __init__(self, root, robot_control):
		self.root = root
		self.robot_control = robot_control
		
		canvas = tk.Canvas(root, bg="white", width=300, height=250)
		canvas.pack(expand=1, fill='both')
		canvas.create_rectangle(175, 175, 125, 125, fill="green")

		b1 = tk.Button(root, text='Go')
		b1.pack()
		b1.bind('<Button-1>', self.startProg)

		b2 = tk.Button(root, text='Exit')
		b2.pack()
		b2.bind('<Button-1>', self.stopProg)
		return
	
	def startProg(self, event=None):
		self.robot_control.go = True
		return

	def stopProg(self, event=None):
		self.robot_control.done = True		
		self.root.quit() 	# close window
		return

def main():
    gMaxRobotNum = 1 # max number of robots to control
    comm = RobotComm(gMaxRobotNum)
    comm.start()
    print 'Bluetooth starts'

    robot_list = comm.robotList
    behaviors = RobotBehavior(robot_list)

    frame = tk.Tk()
    GUI(frame, behaviors)
    frame.mainloop()

    comm.stop()
    comm.join()
    return

if __name__ == "__main__":
    sys.exit(main())