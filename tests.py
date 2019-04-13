#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fichier de tests
"""
import main as mn
import numpy as np

# Test adding atoms

def test1(i):
    empty = np.zeros((3 * ratio, 2 * ratio))
    mn.addElement(mn.Atoms[i], empty)
    mn.draw(np.array([]), empty)
    
def test2():
    for i in range(len(mn.Atoms)):
        test1(i)

# Test findPath
        
def test3(i):
    empty = mn.initializeTable(mn.atomsDisposition, mn.ourRobot, mn.opponentFirstRobot, mn.opponentSecondRobot)
    mn.draw(mn.findPath(empty, mn.Atoms[i]), empty)


def test4():
    for i in range(len(mn.Atoms)):
        test3(i)

# Test getThetaFromSourceToTarget

def test5():
    print(mn.getThetaFromSourceToTarget((75,150),(75,100)))


# Test turnRight / TurnLeft


# Test updateOurRobotPosition

def test6():
    print(mn.ourRobot.getX(), mn.ourRobot.getY(), mn.ourRobot.getDir())
    mn.updateOurRobotPosition(0.2, np.pi / 2 , mn.ourRobot)
    print(mn.ourRobot.getX(), mn.ourRobot.getY(), mn.ourRobot.getDir())


# Test getExpXY
    
def test7():
    element = mn.atom(1, 1.05 + mn.atomDiameter*1.5, 1)
    mn.ourRobot.setDir(np.pi / 2)
    x, y = mn.getExpXY([element, 3, 2, 0.03, 0.03], mn.ourRobot)
    print(round(x,3),round(y,3))
    print(element.getDiameter() / 2 + mn.ourRobot.getDiameter() / 2 + mn.ourRobot.getX() )
    print(mn.ourRobot.getX(), mn.ourRobot.getY())

# TESTS

#test1(17)
#test2()
#test4() 
#test3(14)
#test6()

#action()