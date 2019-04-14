#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This is the code that allows the connection to the robot
"""
import numpy as np

def turnRight(theta):
    # TODO
    res = int((2 * np.pi - theta) * 1000)
    if res == 0:
        return "0000"
    elif res / 10 == 0:
        return "000"+ str(res)
    elif res / 100 == 0:
        return "00"+ str(res)
    elif res / 1000 == 0:
        return "0"+ str(res)
    else:
        return str(res)

def turnLeft(theta):
    # TODO
    res = int(theta * 1000)
    if res == 0:
        return "0000"
    elif res / 10 == 0:
        return "000"+ str(res)
    elif res / 100 == 0:
        return "00"+ str(res)
    elif res / 1000 == 0:
        return "0"+ str(res)
    else:
        return str(res)


def moveForward(d):
    # TODO
    intD = int(d*1000)
    res = "1000000000000"
    
    if intD == 0:
        return res + "0000"
    elif intD // 10 == 0:
        return res + "000"+ str(intD)
    elif intD // 100 == 0:
        return res + "00"+ str(intD)
    elif intD // 1000 == 0:
        return res + "0"+ str(intD)
    else:
        return res + str(intD)

def grab():
    # TODO
    return

def putDown():
    # TODO
    return