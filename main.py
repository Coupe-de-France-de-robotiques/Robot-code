#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This will be the main program implementing the mouvement and path finding algorithm

"""
import numpy as np
import time
import path_finding_algorithm as pfa
import connect_robot as cr
import connect_camera as cc
import matplotlib.pyplot as plt
import sys
import itertools
import random
import copy as cp


#################################################### Global variables ####################################################


# TODO : SET VALUES
camera = False
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

class element:
    def __init__(self, x, y, diameter, label):
        self.__x = round(x, 3)
        self.__y = round(y, 3)
        self.__diameter = round(diameter, 3)
        self.__label = label # use "theirs1" for opponenet's first robot and "theirs2" for their second one if there is one    
    def __str__(self):
        return str(self.__label) + "(x = " + str(self.__x) + ", y = " + str(self.__y) + ")"
    def getX(self):
        return self.__x
    def getY(self):
        return self.__y
    def getDiameter(self):
        return self.__diameter
    def getLabel(self):
        return self.__label;
    def setX(self, x):
        self.__x = round(x,3)
    def setY(self, y):
        self.__y = round(y,3)
    def setLabel(self, label):
        self.__label = label

class robot (element):
    def __init__(self, x, y, diameter, v, theta, label): # for label use "theirs1" for opponenet's first robot and "theirs2" for their second one if there is one
        element.__init__(self, x, y, diameter, label)
        self.__v = round(v, 3)
        self.__theta = round(theta, 3)
    def getDir(self):
        return self.__theta
    def setDir(self, theta):
        self.__theta = round(theta, 3)
    def setDiameter(self, diameter):
        self.__diameter = round(diameter, 3)

class atom (element):
    def __init__(self, x, y, label, diameter = atomDiameter):
        global Atoms
        global idIncrement
        element.__init__(self, x, y, diameter, label)
        self.__id = idIncrement
        idIncrement += 1
        Atoms.append(self)
    def getId(self):
        return self.__id
    def setId(self, Id):
        self.__id = Id
    def setDiameter(self, diameter):
        self.__diameter = round(diameter,3)
    
class point (element):
    def __init__(self, x , y):
        element.__init__(self, x, y, 0.0, "Point")
    
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
        
# TODO : SET VALUES
ourRobot = robot(0.225, 0.75, 0.2,0,0, "ours")
margin = ourRobot.getDiameter()/2
opponentFirstRobot = robot(2.75, 0.5, 0.2,0,0,"theirs1")
opponentSecondRobot = robot(2.75,1,0.1,0,0,"theirs2")
if numberOfOpponentRobots == 1 : opponentSecondRobot = None

atomsDisposition = np.array( #red (1) green (2) bleu(3) ourGoldonium(-1) thierGoldonium(-2) START FROM ATOMS WE WILL NOT USE TO ATOMS OF OPPONENET
        [
                
        ##### primary atoms #####
        
         #near table of elements : ours
        atom(0.5, 0.45, 1),
        atom(0.5, 0.75, 1),
        atom(0.5, 1.05, 2),
        
        
         #in the central cercles : ours
        atom(1, 1.05 + atomDiameter*1.5, 1),
        atom(1, 1.05 - atomDiameter*1.5, 3),
        atom(1 - atomDiameter*1.5 , 1.05, 2),
        atom(1 + atomDiameter*1.5 , 1.05, 1),
        
        
         #in the central cercles : theirs
        atom(2 - atomDiameter/2 , 1.05 - atomDiameter*1.5, 1),
        atom(2 - atomDiameter/2 , 1.05, 2),
        atom(2 + atomDiameter/2 , 1.05, 3),
        atom(2 , 1.05 + atomDiameter, 1),
        
         #near table of elements : theirs
        atom(2.5 , 0.45, 1),
        atom(2.5, 0.75, 1),
        atom(2.5, 1.05, 2),
        
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

# TODO : ADD SCORES AND UPDATE POSITIONS AND START FROM ATOMS WE WILL NOT USE TO ATOMS OF OPPONENET
missions = np.array([ #[atom, target , stage, score]
        [Atoms[0], point(0.225, 0.45), 1, 0], # 1 is for the first part of the mission (robot->atom) and 2 for the second (robot+atom -> target) than 3 for done
        [Atoms[1], point(0.225, 0.45), 1 , 0],
        [Atoms[2], point(0.225, 0.45), 1 , 0],
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
        if np.sqrt(np.power(x - element.getX()*ratio,2) + np.power(y - element.getY()*ratio,2)) <= (element.getDiameter() / 2. + margin) * ratio:
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



def getThetaFromSourceToTarget(source, target): # in ]-pi,pi]
    
    dx = target[0] - source[0]
    dy = target[1] - source[1]
    
    sin = dy/np.sqrt(dx**2 + dy**2)
    tetaFromSource = np.arcsin(sin)
    if(dy >= 0 and dx < 0) : tetaFromSource = np.pi - tetaFromSource
    if(dy < 0 and dx < 0) : tetaFromSource = - np.pi - tetaFromSource
    return tetaFromSource



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


def updateTable(table, ourRobot, opponentFirstRobot, opponentSecondRobot, atomsDisposition, response): # Updates is done if element is at least 7cm near the robot

    """
    to get the current disposition of atoms what we will do is the following:
        1 - get captors response from "response"
        2 - gess what the robot is looking at:
            1-1 - if looking at Robot : update opponentFirstRobot position/orientation or opponentSecondRobot position/orientation
            1-2 - if an obstacle : adjust our robot's position/orientation if possible or do nothing if not
            1-3 - if an atom : guess (randomly & smartly) which atom it is
            (it is not crucial that we do not mistake the choice but the more we mistake the less efficient the program becomes)
        3 - adjust our robot's position/orientation using data from encoders "to be got from response too" and from camera if possible ...
        4 - if the camera is trustfull update all elements in the table
        
    """
    global camera
    
    if camera:
        x,y = cc.getOpponentsFirstRobotPosition()
        opponentFirstRobot.setX(x)
        opponentFirstRobot.setY(y)
        if numberOfOpponentRobots == 2:
            x,y = cc.getOpponentsSecondRobotPosition()
            opponentSecondRobot.setX(x)
            opponentSecondRobot.setY(y)
        x,y = cc.getRobotPosition()
        ourRobot.setX(x)
        ourRobot.setY(y)
        for i in range(len(Atoms)):
            x,y = cc.getAtomPosition(i)
            Atoms[i].setX(x)
            Atoms[i].setY(y)
    else:
        updateOurRobotPosition(getTraveledDistance(response), getRotationAngle(response), ourRobot)
        elements = theRobotIsLookingAt(getCaptorsData(response))
        for elementData in elements:
            updatePosition(elementData, ourRobot, table)

    message("Table state updated...")
    


def theRobotIsLookingAt(captorsData, R = ourRobot.getDiameter() / 2):
    validCaptors = []
    if captorsData[0] <= 0.07 and captorsData[0] != 0.0 and captorsData[1] == 0.0:
        validCaptors.append((0,0))
    if captorsData[5] <= 0.07 and captorsData[5] != 0.0 and captorsData[4] == 0.0:
        validCaptors.append((5,5))
    for i in range(5):
        if captorsData[i] != 0.0 and captorsData[i+1] != 0.0 and func(captorsData[i], captorsData[i+1], R):
            validCaptors.append((i,i+1))       
        
    if len(validCaptors) != 0:
        elements = []
        tmpListOfAtoms = [missions[i][0] for i in range(len(missions)) if missions[i][2] == 1 and missions[i][0].getLabel() >= 0 and missions[i][0].getDiameter() > 0.] # enlever goldonium aussi ?? et les atoms de D=0
        for i,j in validCaptors:
            if len(tmpListOfAtoms) > 0: 
                element = tmpListOfAtoms.pop()
                if i != j:
                    elements.append([element, i, j, captorsData[i], captorsData[j]])
                else:
                    elements.append([element, i, j, captorsData[i], captorsData[j] + element.getDiameter()])
        return elements
    else:
        return []    
        

def updatePosition(elementData, ourRobot, table):

    X, Y = getExpXY(elementData, ourRobot)
    
    oldElement = cp.copy(elementData[0])
    elementData[0].setX(X) #do not forget to update on the table too
    elementData[0].setY(Y)
    updateElement(oldElement, elementData[0], table)



def func(x, y, R):
    rad27 = 0.471238898
    return (y * np.sin(rad27) + R * np.sin(rad27))**2 + (x + R - y * np.cos(rad27) - R * np.cos(rad27))**2 <= 0.08**2



def getExpXY(elementData, ourRobot, pltShow = False, R = ourRobot.getDiameter() / 2):
    # look at the drawing
    r = elementData[0].getDiameter() / 2
    thetas = np.array([-1.178, -0.707, -0.236, 0.236, 0.707, 1.178]) + ourRobot.getDir()
    xA, yA = (R + elementData[3]) * np.cos(thetas[elementData[1]]) + ourRobot.getX(), (R + elementData[3]) * np.sin(thetas[elementData[1]]) + ourRobot.getY()
    xB, yB = (R + elementData[4]) * np.cos(thetas[elementData[2]]) + ourRobot.getX(), (R + elementData[4]) * np.sin(thetas[elementData[2]]) + ourRobot.getY()
    
    if pltShow:
        plt.gca().set_aspect('equal', adjustable='box')
        plt.plot([ourRobot.getX(), ourRobot.getX() + R, xA , xB ],[ourRobot.getY(), ourRobot.getY(), yA, yB], 'ro')
    
    d = np.sqrt((xA-xB)**2 + (yA-yB)**2)
    h = np.sqrt(r**2 - (d/2)**2)
    if xA == xB:
        dx, dy = h, 0
    elif yA == yB:
        dx, dy = 0, h
    else:
        dx, dy = h / np.sqrt(1 + ((xA-xB)/(yA-yB))**2), h / np.sqrt(1 + ((yA-yB)/(xA-xB))**2)
        
    signe = 1
    if (yA-yB) > 0 and (xA-xB) > 0 or (yA-yB) < 0 and (xA-xB) < 0: 
        signe = -1
    x1, y1 = (xA + xB) / 2 + signe * dx, (yA + yB) / 2 + dy
    x2, y2 = (xA + xB) / 2 - signe * dx, (yA + yB) / 2 - dy
    if np.sqrt((x1-ourRobot.getX())**2 + (y1-ourRobot.getY())**2) < np.sqrt((x2-ourRobot.getX())**2 + (y2-ourRobot.getY())**2):
        return x2, y2
    else:
        return x1, y1

        

def updateOurRobotPosition(traveledDistance, rotationAngle, ourRobot): 
        
    theta = rotationAngle + ourRobot.getDir()
    X = ourRobot.getX() + traveledDistance * np.cos(theta)
    Y = ourRobot.getY() + traveledDistance * np.sin(theta)
        
    ourRobot.setX(X)
    ourRobot.setY(Y)
    ourRobot.setDir(theta)



def sendNextActions(table):
    
    """
    given tableDisposition and currMission we do:
        1 - if currentMission is feasible do:
            1-1 - if the next step leads to the goal (we are one step away from the target):
                1-1-1 - increment score
            1-2 - else
                1-1-1 - set score to zero
            1-3 - send next actions to arduino
            1-4 - store arduino's response in response
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
        
        # preceed tests
        if missions[tmpMission][2] == 1:
            path = findPath(table, missions[tmpMission][0])
        elif missions[tmpMission][2] == 2:
            path = findPath(table, missions[tmpMission][1])

    currentMission = tmpMission
    
    
    #sending actions
    
    #proceed with the first part of the path
    firstPart = [(int(ourRobot.getX()*ratio),int(ourRobot.getY()*ratio)), path[0]]
    for i in range(1,len(path)):
        if (path[i][0] - firstPart[i][0] != firstPart[i][0] - firstPart[i-1][0]) or (path[i][1] - firstPart[i][1] != firstPart[i][1] - firstPart[i-1][1]): break
        firstPart.append(path[i])
            
        #rotation
    theta = getThetaFromSourceToTarget((int(ourRobot.getX()*ratio),int(ourRobot.getY()*ratio)), path[0])
    if ourRobot.getDir() > theta:
        cr.turnRight(ourRobot.getDir() - theta)
    elif ourRobot.getDir() < theta:
        cr.turnLeft(ourRobot.getDir() - theta)
    
        #translation
    response = cr.moveForward(np.sqrt(np.power(path[-1][0] - int(ourRobot.getX()*ratio) , 2) + np.power(path[-1][1] - int(ourRobot.getY()*ratio) , 2)) / float(ratio))
    
    if len(path) == 1 and missions[currentMission][2] == 1 and actionComplete(response):
        cr.grab()
    
    score = 0
    if len(path) == 1 and missions[currentMission][2] == 2 and actionComplete(response):
        score = missions[currentMission][3]
    
    #response = "99999999900000000" # Arduino's response after sending action
    return score, response # score != 0 only if the next action is the final action of an operation and response is the string sent by arduino

def actionComplete(response): # returns a boolean saying if response indicates that the action was interupted (false) or not (true)
    return int(response[0]) == 1

def getCaptorsData(response): # 0 if element is out of [0,15]
    return int(response[1:3]) / 100. , int(response[3:5]) / 100. , int(response[5:7]) / 100. , int(response[7:9]) / 100. , int(response[9:11]) / 100. , int(response[11:13]) / 100.  

def getTraveledDistance(response):
    return int(response[13:17]) / 1000.

def getRotationAngle(response):
    if int(response[17:21]) <= np.pi: 
        return int(response[17:21]) / 1000.
    else:
        return - round((2*np.pi - int(response[17:21]) / 1000.),3)



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


def initialRespose(): # returns initial default response (needed ???)
    # TODO : DEPENDS ON RESPONSE FORM
    return "100000000000000000000" # 1st digit for action complete or not => 6 x 2 digits for captors => 4 digits traveled distance => 4 digits angle

#################################################### Main loop #################################################### 
    

tableDisposition = initializeTable(atomsDisposition, ourRobot, opponentFirstRobot, opponentSecondRobot)

def action():
    
    global currentMission
    
    startTime = time.time()
    actionResponse = initialRespose() #string sent by arduino
    score = 0
    
    while time.time() - startTime < 100:
        
        updateTable(tableDisposition, ourRobot, opponentFirstRobot, opponentSecondRobot, atomsDisposition, actionResponse)
        actionScore, actionResponse = sendNextActions(tableDisposition)
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

    plt.figure(figsize=(7,8))
    plt.imshow(table)
    plt.show()

#action()