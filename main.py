#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This will be the main program implementing the mouvement and path finding algorithm

"""
import numpy as np
import path_finding_algorithm as pfa
import connect_vrep as cv


############# Global variables #############
# all dimentions will be in meters
ratio = 100 #means 1m = ratio points in the matrix
atomDiameter = 0.08
maxRobotDiameter = 0.2
numberOfOpponentRobots = 1


############# classes ################
# add orientation? 
class robot:
    def __init__(self, x, y, diameter, v, label="ours"):
        self.x = x
        self.y = y
        self.diameter = diameter
        self.v = v
        self.label = label # use "theirs1" for opponenet's first robot and "theirs2" for their second one if there is one
    def getX(self):
        return self.x
    def getY(self):
        return self.y
    def getDiameter(self):
        return self.diameter

class atom:
    def __init__(self, x, y, label, diameter = atomDiameter):
        self.x = x
        self.y = y
        self.diameter = diameter
        self.label = label
    def getX(self):
        return self.x
    def getY(self):
        return self.y
    def getDiameter(self):
        return self.diameter


############ table and static obstacles #############
emptyTable = np.zeros((3 * ratio, int((2+atomDiameter) * ratio)))

#adding static obstacles:
 #adding central balance obstacle
for x in range(int((1.48 - maxRobotDiameter/2)*ratio), int((1.52 + maxRobotDiameter/2)*ratio)):
    for y in range(int((1.4 - maxRobotDiameter/2)*ratio), int((2+ atomDiameter/2)*ratio)):
        emptyTable[x][y] = 1

 #adding distributors
for x in range(int((0.45 - maxRobotDiameter/2)*ratio), int((2.55 + maxRobotDiameter/2)*ratio)):
    for y in range(int((1.54 - maxRobotDiameter/2)*ratio), int((1.6 + maxRobotDiameter/2)*ratio)):
        emptyTable[x][y] = 1

############ adding robots and atoms ###############
        
def addElement(element, table, margin): #returns 1 if successful and -1 if not
    Xstart = int((element.getX() - element.getDiameter()/2)*ratio)
    Xend = int((element.getX() + element.getDiameter()/2)*ratio)
    Ystart = int((element.getY() - element.getDiameter()/2)*ratio)
    Yend = int((element.getY() + element.getDiameter()/2)*ratio)
    #first test if possible to add the robot if not return -1
    if Xstart < 0 or Xend >= len(table) and Ystart < 0 and Yend >= len(table[0]):
        return -1
    else:
        #if possible add check which margins can fit in
        if int((element.getX() - element.getDiameter()/2)*ratio - margin*ratio) >= 0 : Xstart = int((element.getX() - element.getDiameter()/2)*ratio - margin*ratio)
        else : Xstart = 0
        if int((element.getX() + element.getDiameter()/2)*ratio + margin*ratio) < len(table) : Xend = int((element.getX() + element.getDiameter()/2)*ratio + margin*ratio)
        else : Xend = len(table) -1
        if int((element.getY() - element.getDiameter()/2)*ratio - margin*ratio) >= 0 : Ystart = int((element.getY() - element.getDiameter()/2)*ratio - margin*ratio)
        else : Ystart = 0
        if int((element.getY() + element.getDiameter()/2)*ratio + margin*ratio) < len(table[0]) : Yend = int((element.getY() + element.getDiameter()/2)*ratio + margin*ratio)
        else : Yend = len(table[0]) -1
        #than add the element
        
        for x in range(Xstart, Xend+1):
            for y in range(Ystart, Yend+1):
                table[x][y] = 1
        return 1
    
def addAtoms(atoms, table, margin): #returns 1 if successful and -1 if not
    oldCopy = table
    for atom in atoms:
        if addElement(atom, table, margin) == -1:
            table = oldCopy
            return -1
    return 1
 

############ action #############

 #initialization
ourRobot = robot(1,2,0.2,0)
theirFirstRobot = robot(1.5,1,0.2,0,"theirs1")
theirSecondRobot = robot(2.5,1,0.1,0,"theirs2")
initailAtomsDisposition = np.array( #red (1) green (2) bleu(3) ourGoldonium(-1) thierGoldonium(-2)
        [
        #main atoms
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
        
        #secondary atoms
         #in the two distributors
        atom(0.45+ atomDiameter/2, 0, 1),
        atom(0.45+ atomDiameter*(3/2), 0, 2),
        atom(0.45+ atomDiameter*(5/2), 0, 1),
        atom(0.45+ atomDiameter*(7/2), 0, 3),
        atom(0.45+ atomDiameter*(9/2), 0, 1),
        atom(0.45+ atomDiameter*(11/2), 0, 2),
        
        atom(2.55- atomDiameter/2, 0, 1),
        atom(2.55- atomDiameter*(3/2), 0, 2),
        atom(2.55- atomDiameter*(5/2), 0, 1),
        atom(2.55- atomDiameter*(7/2), 0, 3),
        atom(2.55- atomDiameter*(9/2), 0, 1),
        atom(2.55- atomDiameter*(11/2), 0, 2),
        
         #in the wall
        atom(0.075+ atomDiameter/2, 2, 3),
        atom(0.075+ atomDiameter*(3/2), 2, 2),
        atom(0.075+ atomDiameter*(5/2), 2, 1),
        
        atom(2.925- atomDiameter/2, 2, 1),
        atom(2.925- atomDiameter*(3/2), 2, 2),
        atom(2.925- atomDiameter*(5/2), 2, 3),
        
         #the two near the experiance
        atom(1.3, 0, 3),
        atom(1.7, 0, 3), 
        
        #Goldonium
        atom(0.75, 0, -1),
        atom(2.25, 0, -2)
        ]
        )
table = emptyTable.copy()
"""
todo = np.array(
        [
        [x,y,label],
        [],
        [],
        [],
        [],
        ]
        )
"""


#if addElement(ourRobot, table, ourRobot.getDiameter()/2) == -1 : print("ERROR WHILE ADDING OUR ROBOT TO THE TABLE")

if addElement(theirFirstRobot, table, ourRobot.getDiameter()/2) == -1 : print("ERROR WHILE ADDING THEIR FIRST ROBOT TO THE TABLE")
if numberOfOpponentRobots == 2:
    if addElement(theirSecondRobot, table, ourRobot.getDiameter()/2) == -1 : print("ERROR WHILE ADDING THEIR SECOND ROBOT TO THE TABLE")
if addAtoms(initailAtomsDisposition, table, ourRobot.getDiameter()/2)  == -1 : print("ERROR WHILE ADDING ATOMS TO THE TABLE")

#deleting table margins
table = table[int(maxRobotDiameter/2 *ratio):len(table-int(maxRobotDiameter/2 *ratio)),int(maxRobotDiameter/2 *ratio):len(table[0]-int(maxRobotDiameter/2 *ratio)) ]


 # action
 
 
 
 # pfa 
astar = pfa.astar(table, (12,93), (240,185))
for i,j in astar:
    table[i,j] = 7
    
    
# writing result in a file
f = open('test.txt', "w")
for i in range(len(table)):
    for j in range(len(table[0])):
        f.write(str(int(table[i,j])))
    f.write("\n")
    