#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fichier de tests
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

# Test findPath
        
def test4_sub(i):
    empty = mn.initializeTable(mn.atomsDisposition, mn.ourRobot, mn.opponentFirstRobot, mn.opponentSecondRobot)
    mn.ourRobot.setX(0.5)
    mn.ourRobot.setY(1.75)
    mn.ourRobot.setDir(-2.357)
    mn.draw(mn.findPath(empty, mn.Atoms[i]), empty)


def test4():
    for i in range(len(mn.Atoms)):
        test4_sub(i)

# Test getThetaFromSourceToTarget

def test5():
    print(mn.getThetaFromSourceToTarget((75,150),(75,100)))


# Test turnRight / TurnLeft


# Test updateOurRobotPosition

def test6():
    print(mn.ourRobot.getX(), mn.ourRobot.getY(), mn.ourRobot.getDir())
    mn.updateOurRobotPosition(0.2, np.pi / 2 , mn.ourRobot)
    print(mn.ourRobot.getX(), mn.ourRobot.getY(), mn.ourRobot.getDir())
    mn.updateOurRobotPosition(-0.2, - np.pi / 2 , mn.ourRobot)


# Test getExpXY
    
def test7():
    element = mn.atom(1, 1.05 + mn.atomDiameter*1.5, 1)
    #mn.ourRobot.setDir(np.pi / 2)
    #x, y = mn.getExpXY([element, 0, 1, 0.0, 0.0], mn.ourRobot)
    x, y = mn.getExpXY([element, 3, 4, 0.05, 0.07], mn.ourRobot, True)
    #x, y = mn.getExpXY([element, 2, 3, 0.0, 0.], mn.ourRobot)
    #x, y = mn.getExpXY([element, 3, 4, 0.0, 0.], mn.ourRobot)
    #x, y = mn.getExpXY([element, 4, 5, 0.0, 0.], mn.ourRobot)

    #x1, y1 = mn.getExpXY([element, 0, 0, 0.02, 0.02 + element.getDiameter()], mn.ourRobot)
    #print(round(x,3),round(y,3))
    #print(element.getDiameter() / 2 + mn.ourRobot.getDiameter() / 2 + mn.ourRobot.getX() )
    #print(mn.ourRobot.getX(), mn.ourRobot.getY())


# TEST updatePosition

def test8():
    table = mn.initializeTable(mn.atomsDisposition, mn.ourRobot, mn.opponentFirstRobot, mn.opponentSecondRobot)
    element = mn.atom(1, 1.05 + mn.atomDiameter*1.5, 1)
    for i in range(5):
        elementData = [element, i, i+1, 0.03, 0.03]
        mn.updatePosition(elementData, mn.ourRobot, table)
        mn.draw(np.array([]), table)


# Test theRobotIsLookingAt

def test9():
    response = "000000206000507005759"
    result = mn.theRobotIsLookingAt(mn.getCaptorsData(response))
    print(result)
   
# Test updateTable

def test10():
    table = mn.initializeTable(mn.atomsDisposition, mn.ourRobot, mn.opponentFirstRobot, mn.opponentSecondRobot)
    response = "000000206000507005759"
    mn.updateTable(table, mn.ourRobot, mn.opponentFirstRobot, mn.opponentSecondRobot, mn.atomsDisposition, response)   
    mn.draw(np.array([]), table)

# Test accessibleNeighboors

def test11():
    table = mn.initializeTable(mn.atomsDisposition, mn.ourRobot, mn.opponentFirstRobot, mn.opponentSecondRobot)
    dx, dy = 50, 20
    start = [(100, 50), None]
    print(pfa.accessibleNeighboors(start, table, dx, dy))
    
# Test algorithm
    
def test12():
    table = mn.initializeTable(mn.atomsDisposition, mn.ourRobot, mn.opponentFirstRobot, mn.opponentSecondRobot)
    start, end = (22, 75), (50,60)
    path = pfa.algorithm(table, start, end)    
    print(path)
    mn.draw(np.array(path), table)
        
# TESTS

#test1(17)
#test2()
#test3(17)
test4() 
#test4_sub(14)
#test6()
#test7()
#test8()
#test9()
#test10()
#test12()

#mn.action()