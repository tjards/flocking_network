#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 28 20:29:59 2020

@author: tjards
"""
import numpy as np 

def buildWall(wType, pos): 
    
    if wType == 'horizontal':
        
        # define 3 points on the plane (this one is horizontal)
        wallp1 = np.array([0, 0, pos])
        wallp2 = np.array([5, 10, pos+0.1])
        wallp3 = np.array([20, 30, pos])       
        # define two vectors on the plane
        v1 = wallp3 - wallp1
        v2 = wallp2 - wallp1
        # compute vector normal to the plane
        wallcp = np.cross(v1, v2)
        walla, wallb, wallc = wallcp
        walld = np.dot(wallcp, wallp3)
        walls = np.zeros((6,1)) 
        walls[0:3,0] = np.array(wallcp, ndmin=2)#.transpose()
        walls[3:6,0] = np.array(wallp1, ndmin=2)#.transpose()
        walls_plots = np.zeros((4,1))
        walls_plots[:,0] = np.array([walla, wallb, wallc, walld])
        
        
    if wType == 'vertical':
        
        # define 3 points on the plane (this one is horizontal)
        wallp1 = np.array([0, pos, 0])
        wallp2 = np.array([5, pos+0.1, 10])
        wallp3 = np.array([20,pos, 30])       
        # define two vectors on the plane
        v1 = wallp3 - wallp1
        v2 = wallp2 - wallp1
        # compute vector normal to the plane
        wallcp = np.cross(v1, v2)
        walla, wallb, wallc = wallcp
        walld = np.dot(wallcp, wallp3)
        walls = np.zeros((6,1)) 
        walls[0:3,0] = np.array(wallcp, ndmin=2)#.transpose()
        walls[3:6,0] = np.array(wallp1, ndmin=2)#.transpose()
        walls_plots = np.zeros((4,1))
        walls_plots[:,0] = np.array([walla, wallb, wallc, walld])
                
    return walls, walls_plots