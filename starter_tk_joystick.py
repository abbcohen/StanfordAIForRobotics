'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   Name:          Joystick for Hamster
   By:            Qin Chen
   Last Updated:  5/10/18

   PROPRIETARY and CONFIDENTIAL
   ========================================================================*/
'''
import sys
import Tkinter as tk
from HamsterAPI.comm_ble import RobotComm
#for PC, need to import from commm_usb

class Robots(object):
    leftprox=100
    rightprox=100
    leftfloorcolor=0
    rightfloorcolor=0
    def __init__(self, robotList):
        self.robotList = robotList
        return

    def move_forward(self, event=None):
        if self.robotList:
            for robot in self.robotList:
                robot.set_wheel(0,100)
                robot.set_wheel(1,100)
        else:
            print "waiting for robot"

    def move_backward(self, event=None):
        if self.robotList:
            for robot in self.robotList:
                robot.set_wheel(0,-100)
                robot.set_wheel(1,-100)
        else:
            print "waiting for robot"

    def move_left(self, event=None):
        if self.robotList:
            for robot in self.robotList:
                robot.set_wheel(0,-20)
                robot.set_wheel(1,100)
        else:
            print "waiting for robot"

    def move_right(self, event=None):
        if self.robotList:
            for robot in self.robotList:
                robot.set_wheel(0,100)
                robot.set_wheel(1,-20)
        else:
            print "waiting for robot"

    def get_prox(self, event=None):
        if self.robotList:
            for robot in self.robotList:
                if(robot.get_proximity(0)==0):
                    self.leftprox=100
                else:
                    self.leftprox=robot.get_proximity(0)*1.3
                
                if(robot.get_proximity(1)==0):
                    self.rightprox=100
                else:
                    self.rightprox=robot.get_proximity(1)*1.3
        pass

    def get_floor(self, event=None):
        if self.robotList:
            for robot in self.robotList:
                self.leftfloorcolor=robot.get_floor(0)
                self.rightfloorcolor=robot.get_floor(1)
        pass

    def stop_move(self, event=None):
       if self.robotList:
            for robot in self.robotList:
                robot.set_wheel(0,0)
                robot.set_wheel(1,0)

    def reset_robot(self, event=None): # use Hamster API reset()
        robot.reset()

class UI(object):
    def __init__(self, root, robot_handle):
        self.root = root
        self.robot_handle = robot_handle  # handle to robot commands
        self.canvas = None
        self.prox_l_id = None
        self.prox_r_id = None
        self.floor_l_id = None
        self.floor_r_id = None
        self.initUI()
        return

    def initUI(self):
        ###################################################################
        # Create a Hamster joystick window which contains
        # 1. a canvas widget where "sensor readings" are displayed
        # 2. a square representing Hamster
        # 3. 4 canvas items to display floor sensors and prox sensors
        # 4. a button for exit, i.e., a call to stopProg(), given in this class
        # 5. listen to key press and key release when focus is on this window
        ###################################################################
        canvas=tk.Canvas(self.root, width=200, height=200)
        canvas.pack()
        canvas.create_rectangle(60, 100, 140, 180, fill = 'blue')
        l1= canvas.create_line(80, 100, 80, (Robots.leftprox), fill = 'red', width=1)
        l2= canvas.create_line(120, 100, 120, (Robots.rightprox), fill = 'red', width=1)
        leftfloor= canvas.create_rectangle(70, 140, 90, 150, fill = 'white')
        rightfloor= canvas.create_rectangle(110, 140, 130, 150, fill = 'white')
        canvas.pack()
        self.display_sensor(l1, l2, canvas, rightfloor, leftfloor)

        self.root.bind('<KeyPress>', self.keydown)
        self.root.bind('<KeyRelease>', self.keyup)

    
    ######################################################
    # This function refreshes floor and prox sensor display every 100 milliseconds.
    # Register callback using Tkinter's after method().
    ######################################################
    def display_sensor(self, l1, l2, canvas, rightfloor, leftfloor):
        rightvalue= str(hex(self.robot_handle.rightfloorcolor/10))
        leftvalue= str(hex(self.robot_handle.leftfloorcolor/10))

        rightvalue=rightvalue[2:len(rightvalue)]
        leftvalue=leftvalue[2:len(leftvalue)]

       # print(self.robot_handle.rightfloorcolor, self.robot_handle.leftfloorcolor)

        self.robot_handle.get_prox()
        self.robot_handle.get_floor()

        canvas.coords(l1, 80, 100, 80, self.robot_handle.leftprox)
        canvas.coords(l2, 120, 100, 120, self.robot_handle.rightprox)
        canvas.itemconfig(rightfloor, fill = '#' + rightvalue + rightvalue + rightvalue)
        canvas.itemconfig(leftfloor, fill = '#' + leftvalue + leftvalue + leftvalue)

        self.root.after(100, self.display_sensor, l1, l2, canvas, rightfloor, leftfloor)

    ####################################################
    # Implement callback function when key press is detected
    ####################################################
    def keydown(self, event):
        key=event.char
        print key
        if (key == 'w'):
            self.robot_handle.move_forward()
        elif (key == 'a'):
            self.robot_handle.move_left()
        elif (key == 's'):
            self.robot_handle.move_backward()
        elif (key == 'd'):
            self.robot_handle.move_right()

    #####################################################
    # Implement callback function when key release is detected
    #####################################################
    def keyup(self, event):
        self.robot_handle.stop_move()

    def stopProg(self, event=None):
        self.root.quit()    # close window
        self.robot_handle.reset_robot()
        return

def main(argv=None):
    gMaxRobotNum = 1 # max number of robots to control
    comm = RobotComm(gMaxRobotNum)
    comm.start()
    print 'Bluetooth starts'
    robotList = comm.robotList
    

    robot_handle = Robots(robotList)
    m = tk.Tk() #root
    gui = UI(m, robot_handle)

    b2 = tk.Button(m, text='Exit')
    b2.pack(side='left')
    b2.bind('<Button-2>', UI.stopProg)



    m.mainloop()

    comm.stop()
    comm.join()

if __name__== "__main__":
    sys.exit(main())