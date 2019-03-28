#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fichier de tests
"""
import main as mn

# TEST ADD / DELETE / UPDATE

    #test adding atoms
def test1(i):
    empty = np.zeros((3 * ratio, int((2+atomDiameter) * ratio)))
    mn.addElement(mn.Atoms[i], empty)
    mn.draw(np.array([]), empty)
    
def test2():
    for i in range(len(mn.Atoms)):
        test1(i)

#test1(17)
#test2()


# TEST findPath

def test3(i):
    empty = mn.emptyTable.copy()
    mn.addElement(mn.Atoms[i], empty)
    mn.findPath(empty, mn.Atoms[i])
    mn.draw(np.array([]), empty)


def test4():
    for i in range(len(mn.Atoms)):
        test3(i)
#test4() 
#test3(14)

action()