#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path finding algorithm in a graph of nodes
"""

import heapq as hp
import time
import numpy as np
import itertools

# Standard A* path finding algorithm

timeLimit = 1000 # in miliseconds

def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2


def astarV1(array, start, goal):
    
    startTime = time.time()
    
    if array[goal[0]][goal[1]] != 0.0: return False

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    hp.heappush(oheap, (fscore[start], start))
    
    while oheap :
        
        if time.time() - startTime > timeLimit / 1000.: break

        current = hp.heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] != 0:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                hp.heappush(oheap, (fscore[neighbor], neighbor))
                
    return False


def astarV2(array, start, goal):
    
    startTime = time.time()
    
    if array[goal[0]][goal[1]] != 0.0: return False
    
    
    xDiv = divisors(np.abs(start[0] - goal[0]))
    yDiv = divisors(np.abs(start[1] - goal[1]))
   
    for dx, dy in itertools.zip_longest(xDiv, yDiv, fillvalue = 1):
     
        close_set = set()
        came_from = {}
        gscore = {start:0}
        fscore = {start:heuristic(start, goal)}
        oheap = []
    
        hp.heappush(oheap, (fscore[start], start))
        
        while oheap :
            
            if time.time() - startTime > timeLimit / 1000.: break
    
            current = hp.heappop(oheap)[1]
    
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data.append(start)
                return link(data)
    
            close_set.add(current)
            for n in accessibleNeighboors([current], array, dx, dy, goal):
                neighbor = n[0][0], n[0][1]            
                tentative_g_score = gscore[current] + heuristic(current, neighbor)
                if 0 <= neighbor[0] < array.shape[0]:
                    if 0 <= neighbor[1] < array.shape[1]:                
                        if array[neighbor[0]][neighbor[1]] != 0:
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue
                    
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                    
                if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    hp.heappush(oheap, (fscore[neighbor], neighbor))
                
    return False
        
def link(path):
    result = []
    for i in range(len(path) - 1):
        if path[i][0] == path[i+1][0]:
            if path[i][1] <= path[i+1][1]:
                for j in range(path[i][1], path[i+1][1]):
                    result.append((path[i][0], j))
            else:
                for j in range(path[i][1], path[i+1][1], -1):
                    result.append((path[i][0], j))
        else:
            if path[i][0] <= path[i+1][0]:
                for j in range(path[i][0], path[i+1][0]):
                    result.append((j, path[i][1]))
            else:
                for j in range(path[i][0], path[i+1][0], -1):
                    result.append((j, path[i][1]))
    return result
        

def divisors(n):
    if n == 0: return [1]
    nDiv = []
    for d in range(n, 0, -1):
        if n % d == 0:
            nDiv.append(d)
    return nDiv


def accessibleNeighboors(current, array, dx, dy, target):
    neighboors = []
    addNeighboor(current, array, current[0][0], current[0][0] + dx, neighboors, True)
    addNeighboor(current, array, current[0][0] - dx, current[0][0], neighboors, True)
    addNeighboor(current, array, current[0][1], current[0][1] + dy, neighboors, False)
    addNeighboor(current, array, current[0][1] - dy, current[0][1], neighboors, False)
    
    #neighboors.sort(key = lambda t : -distance(t[0], target))
    
    return neighboors

    
def addNeighboor(current, array, start, end, neighboors, onX):
    if ((end < len(array) and onX) or (end < len(array[0]) and not onX)) and start >= 0:
        ok = True
        for i in range(start, end):
            if onX:
                if array[i,current[0][1]] != 0 :
                    ok = False
            else:
                if array[current[0][0], i] != 0:
                    ok = False
        if ok:
            if onX:
                if start == current[0][0]:
                    neighboors.append([(end, current[0][1]), current])
                else:
                    neighboors.append([(start, current[0][1]), current])
            else:
                if start == current[0][1]:
                    neighboors.append([(current[0][0], end), current])
                else:
                    neighboors.append([(current[0][0], start), current])


def algorithm(array, start, end):
    #return astarV1(array, start, end)
    return astarV2(array, start, end)


# gets a birnary matrix (m_i,j in {0,1}) with 1 being an obstacle
# returns list of tuples representing the optimal path