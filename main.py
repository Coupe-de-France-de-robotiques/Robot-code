#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This will be the main program implementing the mouvement and path finding algorithm



when delete keep track of intersctions so that the algorithm do not go through other atoms



"""
import numpy as np
import time
import path_finding_algorithm as pfa
import connect_vrep as cv
import sim_arduino as sa
import matplotlib.pyplot as plt
import sys
import itertools

#################################################### Global variables ####################################################



ratio = 100 #means 1m = ratio points in the matrix
atomDiameter = 0.08 #in meters
maxRobotDiameter = 0.2 #in meters
numberOfOpponentRobots = 2 #to be set at the begining of the game
idIncrement = 0 #to automaticlly set atoms ids
# @@@@@@@@ For sim @@@@@@@@@
currentMission = -1 # ? nothing ; index of an element of the array missions
# @@@@@@@@ End for sim @@@@@@@@@
displayMessages = False # display print in the program of not
Atoms = []

# Classes 

class robot:
    def __init__(self, x, y, diameter, v, theta, label):
        self.x = x
        self.y = y
        self.diameter = diameter
        self.v = v
        self.theta = theta
        self.label = label # use "theirs1" for opponenet's first robot and "theirs2" for their second one if there is one
    def getX(self):
        return self.x
    def getY(self):
        return self.y
    def getDir(self):
        return self.theta
    def getDiameter(self):
        return self.diameter
    def getLabel(self):
        return self.label;

class atom:
    def __init__(self, x, y, label, diameter = atomDiameter):
        global Atoms
        global idIncrement
        self.x = x
        self.y = y
        self.diameter = diameter
        self.label = label
        self.id = idIncrement
        idIncrement += 1
        Atoms.append(self)
    def getX(self):
        return self.x
    def getY(self):
        return self.y
    def getId(self):
        return self.id
    def getDiameter(self):
        return self.diameter
    def getLabel(self):
        return self.label;
    
class point:
    def __init__(self, x , y):
        self.x = x
        self.y = y
    def getX(self):
        return self.x
    def getY(self):
        return self.y
    def getDiameter():
        return 0.0
    
class colors:
    BLACK   = '\033[1;30m'
    RED     = '\033[1;31m'
    GREEN   = '\033[3;2;32m'
    YELLOW  = '\033[1;33m'
    BLUE    = '\033[1;34m'
    MAGENTA = '\033[1;35m'
    CYAN    = '\033[1;36m'
    WHITE   = '\033[1;37m'
    RESET   = '\033[39m'

# more Global Variables
        
ourRobot = robot(0.225, 0.75, 0.2,0,0, "ours")
margin = ourRobot.getDiameter()/2
opponentFirstRobot = robot(2.75, 0.5, 0.2,0,0,"theirs1")
opponentSecondRobot = robot(2.75,1,0.1,0,0,"theirs2")
if numberOfOpponentRobots == 1 : opponentSecondRobot = None

atomsDisposition = np.array( #red (1) green (2) bleu(3) ourGoldonium(-1) thierGoldonium(-2)
        [
                
        ##### primary atoms #####
        
         #near table of elements
        atom(0.5, 0.45, 1),
        atom(0.5, 0.75, 1),
        atom(0.5, 1.05, 2),
        
        atom(2.5 , 0.45, 1),
        atom(2.5, 0.75, 1),
        atom(2.5, 1.05, 2),
        
         #in the central cercles
        atom(1, 1.05 + atomDiameter*1.5, 1),
        atom(1, 1.05 - atomDiameter*1.5, 3),
        atom(1 - atomDiameter*1.5 , 1.05, 2),
        atom(1 + atomDiameter*1.5 , 1.05, 1),
        
        atom(2 - atomDiameter/2 , 1.05 - atomDiameter*1.5, 1),
        atom(2 - atomDiameter/2 , 1.05, 2),
        atom(2 + atomDiameter/2 , 1.05, 3),
        atom(2 , 1.05 + atomDiameter, 1),
        
         #in the balance
        atom(0.834, 1.8, 2), # to 0.83??
        atom(2.166 , 1.8, 2),
        
        
        
        ##### secondary atoms #####
        
         #in the two distributors
        atom(0.45+ atomDiameter/2, 0, 1, 0.),
        atom(0.45+ atomDiameter*(3/2), 0, 2, 0.),
        atom(0.45+ atomDiameter*(5/2), 0, 1, 0.),
        atom(0.45+ atomDiameter*(7/2), 0, 3, 0.),
        atom(0.45+ atomDiameter*(9/2), 0, 1, 0.),
        atom(0.45+ atomDiameter*(11/2), 0, 2, 0.),
        
        atom(2.55- atomDiameter/2, 0, 1, 0.),
        atom(2.55- atomDiameter*(3/2), 0, 2, 0.),
        atom(2.55- atomDiameter*(5/2), 0, 1, 0.),
        atom(2.55- atomDiameter*(7/2), 0, 3, 0.),
        atom(2.55- atomDiameter*(9/2), 0, 1, 0.),
        atom(2.55- atomDiameter*(11/2), 0, 2, 0.),
        
         #in the wall
        atom(0.075+ atomDiameter/2, 2, 3, 0.),
        atom(0.075+ atomDiameter*(3/2), 2, 2, 0.),
        atom(0.075+ atomDiameter*(5/2), 2, 1, 0.),
        
        atom(2.925- atomDiameter/2, 2, 1, 0.),
        atom(2.925- atomDiameter*(3/2), 2, 2, 0.),
        atom(2.925- atomDiameter*(5/2), 2, 3, 0.),
        
         #the two near the experiance
        atom(1.3, 0, 3, 0.),
        atom(1.7, 0, 3, 0.), 
        
        
        
        ##### Goldonium #####
        atom(0.75, 0, -1, 0.),
        atom(2.25, 0, -2, 0.)
        
        ]
        )

missions = np.array([
        [Atoms[0], point(0.225, 0.45), 1], # 1 is for the first part of the mission (robot->atom) and 2 for the second (robot+atom -> target) than 3 for done
        [Atoms[1], point(0.225, 0.45), 1],
        [Atoms[2], point(0.225, 0.45), 1],
        [Atoms[3], point(0.225, 0.45), 1],
        [Atoms[4], point(0.225, 0.45), 1],
        [Atoms[5], point(0.225, 0.45), 1],
        [Atoms[6], point(0.225, 0.45), 1],
        [Atoms[7], point(0.225, 0.45), 1],
        [Atoms[8], point(0.225, 0.45), 1],
        [Atoms[9], point(0.225, 0.45), 1],
        [Atoms[10], point(0.225, 0.45), 1],
        [Atoms[11], point(0.225, 0.45), 1],
        [Atoms[12], point(0.225, 0.45), 1],
        [Atoms[13], point(0.225, 0.45), 1],
        [Atoms[14], point(0.225, 0.45), 1],
        [Atoms[15], point(0.225, 0.45), 1],
        [Atoms[16], point(0.225, 0.45), 1],
        [Atoms[17], point(0.225, 0.45), 1],
        [Atoms[18], point(0.225, 0.45), 1],
        [Atoms[19], point(0.225, 0.45), 1],
        [Atoms[20], point(0.225, 0.45), 1],
        [Atoms[21], point(0.225, 0.45), 1],
        [Atoms[22], point(0.225, 0.45), 1],
        [Atoms[23], point(0.225, 0.45), 1],
        [Atoms[24], point(0.225, 0.45), 1],
        [Atoms[25], point(0.225, 0.45), 1],
        [Atoms[26], point(0.225, 0.45), 1],
        [Atoms[27], point(0.225, 0.45), 1],
        [Atoms[28], point(0.225, 0.45), 1],
        [Atoms[29], point(0.225, 0.45), 1],
        [Atoms[30], point(0.225, 0.45), 1],
        [Atoms[31], point(0.225, 0.45), 1],
        [Atoms[32], point(0.225, 0.45), 1],
        [Atoms[33], point(0.225, 0.45), 1],
        [Atoms[34], point(0.225, 0.45), 1],
        [Atoms[35], point(0.225, 0.45), 1],
        [Atoms[36], point(0.225, 0.45), 1],
        [Atoms[37], point(0.225, 0.45), 1],
        ])


# empty table
emptyTable = np.zeros((3 * ratio, 2 * ratio))

#adding static obstacles:
 #adding central balance obstacle
for x in range(int((1.48 - margin)*ratio), int((1.52 + margin)*ratio)):
    for y in range(int((1.4 - margin)*ratio), 2*ratio):
        emptyTable[x][y] = 1

 #adding distributors
for x in range(int((0.45 - margin)*ratio), int((2.55 + margin)*ratio)):
    for y in range(int((1.54 - margin)*ratio), int((1.6 + margin)*ratio)):
        emptyTable[x][y] = 1
        
 #adding table margins
for x in range(0, 3*ratio):
    for y in range(0, int(margin*ratio)):
        emptyTable[x][y] = 1
    for y in range(2*ratio - int(margin*ratio), 2*ratio):
        emptyTable[x][y] = 1  
for y in range(0, 2*ratio):
    for x in range(0, int(margin*ratio)):
        emptyTable[x][y] = 1
    for x in range(3*ratio - int(margin*ratio), 3*ratio):
        emptyTable[x][y] = 1  
        


#################################################### methods ####################################################



def message(message):
    global displayMessages
    if displayMessages: print(message)


      
def getBorders(element): #gets borders of an element in the matrix given a margin and given the dimensions of the robot
    
    global margin
    Xstart = int((element.getX() - element.getDiameter()/2)*ratio)
    Xend = int((element.getX() + element.getDiameter()/2)*ratio)
    Ystart = int((element.getY() - element.getDiameter()/2)*ratio)
    Yend = int((element.getY() + element.getDiameter()/2)*ratio)
     #first test if possible to add the robot if not return -1
    if Xstart < 0 or Xend >= (3 * ratio) and Ystart < 0 and Yend >= 2 * ratio:
        return -1, -1, -1, -1
    else:
        #if possible add check which margins can fit in
        if int((element.getX() - element.getDiameter()/2)*ratio - margin*ratio) >= 0 : Xstart = int((element.getX() - element.getDiameter()/2)*ratio - margin*ratio)
        else : Xstart = 0
        if int((element.getX() + element.getDiameter()/2)*ratio + margin*ratio) < 3 * ratio : Xend = int((element.getX() + element.getDiameter()/2)*ratio + margin*ratio)
        else : Xend = (3 * ratio) -1
        if int((element.getY() - element.getDiameter()/2)*ratio - margin*ratio) >= 0 : Ystart = int((element.getY() - element.getDiameter()/2)*ratio - margin*ratio)
        else : Ystart = 0
        if int((element.getY() + element.getDiameter()/2)*ratio + margin*ratio) < 2 * ratio : Yend = int((element.getY() + element.getDiameter()/2)*ratio + margin*ratio)
        else : Yend = (2 * ratio) -1
    
    return Xstart, Xend, Ystart, Yend



def addElement(element, table): #returns 1 if successful and -1 if not
    
    global margin
    
    Xstart, Xend, Ystart, Yend = getBorders(element)
    warning = False
    
    if Xstart == -1 : return -1
    
    for x,y in itertools.product(range(Xstart, Xend+1), range(Ystart, Yend+1)):
        if np.sqrt(np.power(x - element.getX()*ratio,2) + np.power(y - element.getY()*ratio,2)) <= (element.getDiameter() / 2. + margin) * ratio:
            if table[x][y] == 1: warning = True
            table[x][y] += 1
            
    if warning :
        if isinstance(element.getLabel(), int):
            sys.stdout.write(colors.RED)
            message("Warning! : Some of the cells used in atom " + str(element.getId()) + " were already occupied")
            sys.stdout.write(colors.RESET)
        else:
            sys.stdout.write(colors.RED)
            message("Warning! : Some of the cells used in the robot " + element.getLabel() + " were already occupied")
            sys.stdout.write(colors.RESET)
    return 1      


def deleteElement(element, table): #returns 1 if successful and -1 if not
    
    global margin
    
    #todo : consider near elements to not delete all !
    
    Xstart, Xend, Ystart, Yend = getBorders(element)
    
    if Xstart == -1 : return -1
    for x,y in itertools.product(range(Xstart, Xend+1), range(Ystart, Yend+1)):
        if np.sqrt(np.power((x - Xstart/2. - Xend/2.),2) + np.power((y - Ystart/2. - Yend/2.),2)) <= ((element.getDiameter()) / 2. + margin) * ratio:
            table[x][y] -= 1
    return 1



def updateElement(oldElement, newElement, table): #returns 1 if successful and -1 if not
    if deleteElement(oldElement, table) == -1 : return -1
    if addElement(newElement, table) == -1 : return -1
    return 1

        
    
def addAtoms(atoms, table): #returns 1 if successful and -1 if not
    
    global margin
    
    oldCopy = table.copy()
    for atom in atoms:
        if addElement(atom, table) == -1:
            table = oldCopy
            return -1
    sys.stdout.write(colors.GREEN)
    message("Ok! Atoms successfuly added")
    sys.stdout.write(colors.RESET)
    return 1



def getThetaFromTargetToSource(target, source): # in ]-pi,pi]
    
    dx = source[0] - target[0]
    dy = source[1] - target[1]
    
    sin = dx/np.sqrt(dx**2 + dy**2)
    tetaFromTarget = np.arcsin(sin)
    if(dy < 0 and dx >= 0) : tetaFromTarget = np.pi - tetaFromTarget
    if(dy < 0 and dx < 0) : tetaFromTarget = - np.pi - tetaFromTarget
    return tetaFromTarget



def initializeTable(atomsDisposition, ourRobot, opponentFirstRobot, opponentSecondRobot):
        
    table = emptyTable.copy()
    
    #if addElement(ourRobot, table) == -1 : message("ERROR WHILE ADDING OUR ROBOT TO THE TABLE")
    
    if addElement(opponentFirstRobot, table) == -1 : message("ERROR WHILE ADDING THEIR FIRST ROBOT TO THE TABLE")
    if opponentSecondRobot != None:
        if addElement(opponentSecondRobot, table) == -1 : message("ERROR WHILE ADDING THEIR SECOND ROBOT TO THE TABLE")
    if addAtoms(atomsDisposition, table)  == -1 : message("ERROR WHILE ADDING ATOMS TO THE TABLE")
    
    #deleting table margins
    #table = table[int(maxRobotDiameter/2 *ratio):len(table-int(maxRobotDiameter/2 *ratio)),int(maxRobotDiameter/2 *ratio):len(table[0]-int(maxRobotDiameter/2 *ratio)) ]
    
    sys.stdout.write(colors.GREEN)
    message("Ok! Table successfuly initialized")
    sys.stdout.write(colors.RESET)
    
    
    return table


def updateTable(table, ourRobot, opponentFirstRobot, opponentSecondRobot, atomsDisposition, response):
    
    """
    to get the current disposition of atoms what we will do is the following:
        1 - get captors response from "response"
        2 - gess what the robot is looking at:
            1-1 - if Robot : update opponentFirstRobot position/orientation or opponentSecondRobot position/orientation
            1-2 - if an obstacle : adjust our robot's position/orientation if possible or do nothing if not
            1-3 - if an atom : guess which atom it is (the nearest one to the robot with the color we see for example or use the camera) 
            anyway if we mistake we will only have two atoms with the same color switched which is not a problem ..
        3 - adjust our robot's position/orientation using data from encoders "to be got from response too" and from camera if possible ...
        NOTE that an initial robot's position/orientation ajustment will be done in the method sendNextAction so that we can raise an error if 
        there is a big difference between theorical and expirimental values !
        
    """
    
    #1
    leftFrontCaptor = int(response[0:3]) / 100. #distance to something in m
    rightFrontCaptor = int(response[3:6]) / 100. #distance to something in m
    backCaptor = int(response[6:9]) / 100. #distance to something in m
    
    #2
    element = theRobotIsLookingAt(leftFrontCaptor, rightFrontCaptor, backCaptor, table)
    if element[0] == "their first robot":
        updatePosition(opponentFirstRobot, leftFrontCaptor, element[1], rightFrontCaptor, backCaptor, table)
    elif element[0] == "their second robot":
        updatePosition(opponentSecondRobot, element[1], leftFrontCaptor, rightFrontCaptor, backCaptor, table)
    else:
        updatePosition(atomsDisposition, element[1], leftFrontCaptor, rightFrontCaptor, backCaptor, table)
        
    #3
    ecodorLeft = int(response[9:12]) / 100. #distance traveled by the left encodor after the action in m
    ecodorRight = int(response[12:15]) / 100. #distance traveled by the left encodor after the action in m
    updateOurRobotPosition(ecodorLeft, ecodorRight, ourRobot)
    
    message("Table state updated...")
    
    return
 
def theRobotIsLookingAt(leftFrontCaptor, rightFrontCaptor, backCaptor, table): #returns the element of the table info
    return ["atom", 5]

def updatePosition(element, ID, leftFrontCaptor, rightFrontCaptor, backCaptor, table): 
    #ask for camera data
    return

def updateOurRobotPosition(ecodorLeft, ecodorRight, ourRobot): # raises error if theorie != experience
    #ask for camera data
    if False : raise ValueError('Error in updating the robot is position!')
    return



def sendNextAction(table):
    
    """
    given tableDisposition and currMission we do:
        1 - if currentMission is feasible do:
            1-1 - if the next step leads to the goal (we are one step away from the target):
                1-1-1 - increment score 
                1-1-2 - send action to arduino
                1-1-3 - store arduino's response in response
            1-2 - else
                1-1-1 - set score to zero
                1-1-2 - send action to arduino
                1-1-3 - store arduino's response in response
        2- look for a feasable mission:
            2-1 - if found:
                change currentMission and go to 1
            2-2 - if not:
                wait some time (1s for example and retry)
                raise an error after (5s for example)
    
    """
    
    global currentMission
    
    #1 / 2
    path = False
    tmpMission = currentMission - 1;
    while isinstance(path, bool):
        tmpMission += 1 # raise error if no mission is possible.. of wait 2-2
        
        # delete atom from table in case we have an atom SO ADD THE TEST HERE @todo
        #deleteElement(missions[tmpMission][0], table)
        
        # todo : robot position should be 1/2*diametter far from atom's pos
        # preceed tests
        if missions[tmpMission][2] == 1:
            path = findPath(table, missions[tmpMission][0])
        else:
            path = findPath(table, missions[tmpMission][1])
            
        #re-add deleted element
        #addElement(missions[tmpMission][0], table)

    currentMission = tmpMission
    
    #1
    
    # @@@@@@@@ For sim @@@@@@@@@
    #time.sleep(1)
    draw(path, table)
    # @@@@@@@@ End for sim @@@@@@@@@
    
    response = "999999999000000" # Arduino's response after sending action
    score = 0
    return score, response # score != 0 only if the next action is the final action of an operation and response is the string sent by arduino


def findPath(table, target):
        
    global margin
    
    border = []
    x = int(target.getX()*ratio)
    y = int(target.getY()*ratio)
    mrg = int((target.getDiameter()/2. + margin)*ratio)
    
    #ordring borders to begin with the neighrest to the robot
    tmp = [(x + mrg + 1, y), (x - mrg - 1, y), (x, y + mrg + 1), (x, y - mrg -1)]
    tmp.sort(key = lambda t : np.power(int(ourRobot.getX()*ratio) - t[0], 2) + np.power(int(ourRobot.getY()*ratio) - t[1], 2))
    
    for i,j in tmp:
        if i < len(table) - 1 and i > 0 and j < len(table[0]) - 1 and j > 0:
            if table[i][j] == 0.:
                border.append((i,j))    
        
    for borderElement in border:
        path = pfa.astar(table, (int(ourRobot.getX()*ratio), int(ourRobot.getY()*ratio)), (int(borderElement[0]), int(borderElement[1])))
        if not isinstance(path, bool):
            return path
    return False

#################################################### Main loop #################################################### 
    

tableDisposition = initializeTable(atomsDisposition, ourRobot, opponentFirstRobot, opponentSecondRobot)

def action():
    
    global currentMission
    
    startTime = time.time()
    actionResponse = "999999999000000" #string sent by arduino
    score = 0
    
    while time.time() - startTime < 100:
        
        # @@@@@@@@ For sim @@@@@@@@@
        tableDisposition = initializeTable(atomsDisposition, ourRobot, opponentFirstRobot, opponentSecondRobot)
        currentMission += 1
        # @@@@@@@@ End for sim @@@@@@@@@
        
        updateTable(tableDisposition, ourRobot, opponentFirstRobot, opponentSecondRobot, atomsDisposition, actionResponse)
        actionScore, actionResponse = sendNextAction(tableDisposition) 
        if actionScore == -1 :
            raise ValueError('Error in actionScore!')
        else:
            score += actionScore
            
            sys.stdout.write(colors.GREEN)
            message("Success! current score is " + str(score) + " pts")
            sys.stdout.write(colors.RESET)
            

#################################################### for tests #################################################### 
   
def draw(path, table):
    for i,j in path:
        table[i,j] = 7

#    with open('test_' + label + '.txt', "w") as f:
#        for i in range(len(table)):
#            for j in range(len(table[0])):
#                f.write(str(int(table[i,j])))
#            f.write("\n")
    plt.figure(figsize=(7,8))
    plt.imshow(table)
    plt.show()


# pfa
# table = initializeTable(atomsDisposition, ourRobot, opponentFirstRobot, opponentSecondRobot)
# astar = pfa.astar(table, (12,90), (240,185))
# for i,j in astar:
#     table[i,j] = 7



# writing result in a file
#f = open('test.txt', "w")
#for i in range(len(table)):
#    for j in range(len(table[0])):
#        f.write(str(int(table[i,j])))
#    f.write("\n")
#f.close()



#action()