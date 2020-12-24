#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 22 11:48:18 2020

Implement a double integrator 


@author: tjards
"""


#%% Imports
from scipy.integrate import ode
import numpy as np
import animation 
import ctrl_flock as flock
import dynamics_node as node

#%% Setup 

# Simulation Parameters
# ----------------
Ti = 0      # initial time
Tf = 30      # final time 
Ts = 0.1    # sample time
nVeh = 20
iSpread = 50

state = np.zeros((6,nVeh))
state[0,:] = iSpread*(np.random.rand(1,nVeh)-0.5)    # position (x)
state[1,:] = iSpread*(np.random.rand(1,nVeh)-0.5)    # position (y)
state[2,:] = iSpread*(np.random.rand(1,nVeh)-0.5)    # position (z)
state[3,:] = 0    # velocity (vx)
state[4,:] = 0    # velocity (vy)
state[5,:] = 0    # velocity (vz)
cmd = np.zeros((3,nVeh))
cmd[0] = np.random.rand(1,nVeh)-0.5      # command (x)
cmd[1] = np.random.rand(1,nVeh)-0.5      # command (y)
cmd[2] = np.random.rand(1,nVeh)-0.5      # command (z)

# initial target positions 
targets = np.vstack(20*(np.random.rand(3,nVeh)-0.5))
#targets = np.vstack((np.ones((3,nVeh))-0.5))*2
targets[0,:] = 10*np.sin(0.02*0)*np.ones((1,nVeh))
targets[1,:] = 10*np.cos(0.03*0)*np.ones((1,nVeh))
targets[2,:] = 10*np.sin(0.04*0)*np.ones((1,nVeh))
targets[0,10:21] = -5*np.sin(0.01*0)*np.ones((1,10))
targets[1,10:21] = -10*np.cos(0.05*0)*np.ones((1,10))
targets[2,10:21] = 7*np.sin(0.02*0)*np.ones((1,10))


error = state[0:3,:] - targets


#%% Run simulation 
# ----------------

# initialize 
t = Ti
i = 1
nSteps = int(Tf/Ts+1)
t_all          = np.zeros(nSteps)
states_all     = np.zeros([nSteps, len(state), nVeh])
cmds_all       = np.zeros([nSteps, len(cmd), nVeh])

# store first steps
t_all[0]                = Ti
states_all[0,:,:]       = state
cmds_all[0,:,:]         = cmd

# run
while round(t,3) < Tf:
  
    # evolve the target
    targets[0,0:10] = 10*np.sin(0.02*i)*np.ones((1,10))
    targets[1,0:10] = 10*np.cos(0.03*i)*np.ones((1,10))
    targets[2,0:10] = 10*np.sin(0.04*i)*np.ones((1,10))
    targets[0,10:21] = -5*np.sin(0.01*i)*np.ones((1,10))
    targets[1,10:21] = -10*np.cos(0.05*i)*np.ones((1,10))
    targets[2,10:21] = 7*np.sin(0.02*i)*np.ones((1,10))
    
  
    # evolve the inputs 
    state = node.evolve(Ts, state, cmd)
    
    # store
    t_all[i]                = t
    states_all[i,:,:]       = state
    cmds_all[i,:,:]         = cmd
    
    #increment 
    t += Ts
    i += 1
    
    # command for the next time step (target)
    cmd, error = flock.controller(Ts, i, state,cmd, nVeh, targets, error)
    
    # flocking part
    states_q = state[0:3,:]
    states_p = state[0:3,:]
    r = 2
    cmd += flock.interactions(states_q, states_p, r)
    
#%% plot

ani = animation.animateMe(Ts, t_all, states_all, cmds_all, targets, nVeh)
#plt.show()    


         