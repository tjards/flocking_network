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


def flock_sum(u_int, u_obs, u_nav):
    
    u_sum = u_int + u_obs + u_nav
    
    return u_sum

#def u_int():
    
    
# compute the sigma norm
# - a la: Reza Olfati-Saber,"Flocking for Multi-Agent Dynamic Systems:
# Algorithms and Theory", IEEE TRANSACTIONS ON AUTOMATIC CONTROL, 
# Vol. 51 (3), 3 Mar 2006
# --------------------------------------------------------------------    
def sigma_norm(z):    
    eps = 0.5
    norm_sig = (1/eps)*(np.sqrt(1+eps*np.linalg.norm(z)**2)-1)
    
# compute n_ij
# ------------
def n_ij(q_i, q_j):
    eps = 0.5
    n_ij = np.divide(q_j-q_i,np.sqrt(1+eps*np.linalg.norm(q_j-q_i)**2))
    
#
