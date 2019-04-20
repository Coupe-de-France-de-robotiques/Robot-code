#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  9 15:37:24 2019

@author: ayoubfoussoul
"""

import main as mn
import numpy as np
import path_finding_algorithm as pfa

# Test adding atoms

def test1(i):
    empty = np.zeros((3 * mn.ratio, 2 * mn.ratio))
    mn.addElement(mn.Atoms[i], empty)
    mn.draw(np.array([]), empty)
    
def test2():
    for i in range(len(mn.Atoms)):
        test1(i)
        
# Test delete atoms

def test3(i):
    empty = np.zeros((3 * mn.ratio, 2 * mn.ratio))
    mn.addElement(mn.Atoms[i], empty)
    mn.deleteElement(mn.Atoms[i], empty)
    mn.draw(np.array([]), empty)

# Test initializeTable
    
def test4():
    empty = mn.initializeTable(mn.atomsDisposition, mn.ourRobot, mn.opponentFirstRobot, mn.opponentSecondRobot)
    mn.draw(np.array([]), empty)
    
# Test findPath
        
def test5(i):
    empty = mn.initializeTable(mn.atomsDisposition, mn.ourRobot, mn.opponentFirstRobot, mn.opponentSecondRobot)
    mn.ourRobot.setX(0.5)
    mn.ourRobot.setY(1.75)
    mn.ourRobot.setDir(-2.357)
    mn.draw(mn.findPath(empty, mn.Atoms[i]), empty)


def test6():
    for i in range(len(mn.Atoms)):
        test5(i)

# Test getThetaFromSourceToTarget

def test7():
    print(mn.getThetaFromSourceToTarget((75,150),(75,100)))


# Test turnRight / TurnLeft


# Test updateOurRobotPosition

def test8():
    print(mn.ourRobot.getX(), mn.ourRobot.getY(), mn.ourRobot.getDir())
    mn.updateOurRobotPosition(0.2, np.pi / 2 , mn.ourRobot)
    print(mn.ourRobot.getX(), mn.ourRobot.getY(), mn.ourRobot.getDir())
    mn.updateOurRobotPosition(-0.2, - np.pi / 2 , mn.ourRobot)


# TEST updatePosition

def test9():
    table = mn.initializeTable(mn.atomsDisposition, mn.ourRobot, mn.opponentFirstRobot, mn.opponentSecondRobot)
    element = mn.atom(1, 1.05 + mn.atomDiameter*1.5, 1)
    for i in range(5):
        elementData = [element, i, i+1, 0.03, 0.03]
        mn.updatePosition(elementData, mn.ourRobot, table)
        mn.draw(np.array([]), table)


# Test getExpXY
    
def test10():
    element = mn.atom(1, 1.05 + mn.atomDiameter*1.5, 1)
    #mn.ourRobot.setDir(np.pi / 2)
    #x, y = mn.getExpXY([element, 0, 1, 0.0, 0.0], mn.ourRobot)
    x, y = mn.getExpXY([element, 3, 4, 0.05, 0.07], mn.ourRobot, True)
    #x, y = mn.getExpXY([element, 2, 3, 0.0, 0.], mn.ourRobot)
    #x, y = mn.getExpXY([element, 3, 4, 0.0, 0.], mn.ourRobot)
    #x, y = mn.getExpXY([element, 4, 5, 0.0, 0.], mn.ourRobot)

    #x1, y1 = mn.getExpXY([element, 0, 0, 0.02, 0.02 + element.getDiameter()], mn.ourRobot)
    print(round(x,3),round(y,3))
    #print(element.getDiameter() / 2 + mn.ourRobot.getDiameter() / 2 + mn.ourRobot.getX() )
    #print(mn.ourRobot.getX(), mn.ourRobot.getY())


# Test theRobotIsLookingAt

def test11():
    response = "000000206000507005759"
    result = mn.theRobotIsLookingAt(mn.getCaptorsData(response))
    print(result)
   
# Test updateTable

def test12():
    table = mn.initializeTable(mn.atomsDisposition, mn.ourRobot, mn.opponentFirstRobot, mn.opponentSecondRobot)
    response = "000000206000507005759"
    mn.updateTable(table, mn.ourRobot, mn.opponentFirstRobot, mn.opponentSecondRobot, mn.atomsDisposition, response)   
    mn.draw(np.array([]), table)

# Test accessibleNeighboors

def test13():
    table = mn.initializeTable(mn.atomsDisposition, mn.ourRobot, mn.opponentFirstRobot, mn.opponentSecondRobot)
    dx, dy = 50, 20
    start = [(100, 50), None]
    print(pfa.accessibleNeighboors(start, table, dx, dy))
    

# TESTS

#test1(17)
#test2()
#test3(17)
#test4() 
#test5(14)
#test6()
#test7()
#test8()
#test9()
#test10()
#test11()
#test12()
#test13()


mn.action()