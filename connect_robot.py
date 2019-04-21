#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  11 15:39:30 2019

@author: ayoubfoussoul
"""

"""
This is the code that allows the connection to the robot

For now these functions return the distances/angles that they were asked to do in the response as if the action was completed witout any problem
You can change the the return value of one of the calls of moveForward(d) for example to be ["0" (means action non complete + "000003040000" + "xyzt"]
where xyzt represent a distance smaller than d means that the robot did not complete the action in this call of the fuction because something was 3cm and 4cm
in front of the sensors 2 and 3 respectively (uncomment corresponding lines to do so)
"""
import numpy as np
import time

def turnRight(theta):
    # TODO
    #time.sleep(theta/10)
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
    #time.sleep(theta/10)
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

callNumber = 0

def moveForward(d):
    # TODO
    #time.sleep(d)
    global callNumber
    
    # Uncomment this to simulate a non complete action
#    if callNumber == 25: # in the 25th call of the fuction return a response saying that the robot has an atom in fron of him and only did a half of the mission
#        intD = int((d/2)*1000) # means only the half of the distance was done
#        res = "0000003040000"
#        callNumber += 1
#        
#        if intD == 0:
#            return res + "0000"
#        elif intD // 10 == 0:
#            return res + "000"+ str(intD)
#        elif intD // 100 == 0:
#            return res + "00"+ str(intD)
#        elif intD // 1000 == 0:
#            return res + "0"+ str(intD)
#        else:
#            return res + str(intD)
        
    
    intD = int(d*1000)
    res = "1000000000000"
    callNumber += 1
    
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