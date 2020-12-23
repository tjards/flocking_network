#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 22 20:11:52 2020

@author: tjards
"""

import numpy as np

def controller(Ts, i, state, cmd, nVeh, targets, error_prev):
    
    
    #generate random targets
    #targets = np.vstack(20*(np.random.rand(3,nVeh)-0.5))
    
    #simple commands
    kp = 1
    kd = 2
    
    error = state[0:3,:] - targets
    derror = (error_prev - error)/Ts
    
    cmd = -kp*error + kd*derror
    
    
    # #generate random inputs
    # if i == 20:
    #     cmd[0] = - 0.5*cmd[0] + np.random.rand(1,nVeh)-0.5      # command (x)
    #     cmd[1] = - 0.5*cmd[0] + np.random.rand(1,nVeh)-0.5      # command (y)
    #     cmd[2] = - 0.5*cmd[0] + np.random.rand(1,nVeh)-0.5      # command (z)
        
    # if i == 40:
    #     cmd[0] = - 1*cmd[0] + np.random.rand(1,nVeh)-0.5      # command (x)
    #     cmd[1] = - 1*cmd[0] + np.random.rand(1,nVeh)-0.5      # command (y)
    #     cmd[2] = - 1*cmd[0] + np.random.rand(1,nVeh)-0.5      # command (z)
        
    return cmd, error