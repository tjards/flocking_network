#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 22 11:48:18 2020

The project implements 3_D flocking on a double-integrator model, as
described in:
    
Olfati-Saber, R, "Flocking for Multi-Agent Dynamic Systems:
Algorithms and Theory", IEEE TRANSACTIONS ON AUTOMATIC CONTROL, 
Vol. 51 (3), 3 Mar 2006

@author: tjards

"""


#%% Import stuff
# --------------

from scipy.integrate import ode
import numpy as np
import animation 
import ctrl_flock as flock
import dynamics_node as node

#%% Simulation Parameters
# -----------------------
Ti = 0          # initial time
Tf = 30         # final time 
Ts = 0.02       # sample time
nVeh = 10       # number of vehicles
nObs = 5        # number of obstacles
iSpread = 2     # initial spread of vehicles 

# Vehicles states
# ---------------
state = np.zeros((6,nVeh))
state[0,:] = iSpread*(np.random.rand(1,nVeh)-0.5)   # position (x)
state[1,:] = iSpread*(np.random.rand(1,nVeh)-0.5)   # position (y)
state[2,:] = iSpread*(np.random.rand(1,nVeh)-0.5)   # position (z)
state[3,:] = 0                                      # velocity (vx)
state[4,:] = 0                                      # velocity (vy)
state[5,:] = 0                                      # velocity (vz)

# Commands
# --------
cmd = np.zeros((3,nVeh))
cmd[0] = np.random.rand(1,nVeh)-0.5      # command (x)
cmd[1] = np.random.rand(1,nVeh)-0.5      # command (y)
cmd[2] = np.random.rand(1,nVeh)-0.5      # command (z)

# Targets
# -------
targets = 4*(np.random.rand(6,nVeh)-0.5)
targets[0,:] = 1
targets[1,:] = 1
targets[2,:] = 1
targets[3,:] = 0
targets[4,:] = 0
targets[5,:] = 0
error = state[0:3,:] - targets[0:3,:]

# Obstacles
# --------
obstacles = np.zeros((4,nObs))
obstacles[0,:] = 5*iSpread*(np.random.rand(1,nObs)-0.5)    # position (x)
obstacles[1,:] = 5*iSpread*(np.random.rand(1,nObs)-0.5)    # position (y)
obstacles[2,:] = 5*iSpread*(np.random.rand(1,nObs)-0.5)    # position (z)
obstacles[3,:] = np.random.rand(1,nObs)+0.5                  # radii of obstacle(s)


#%% Run simulation 
# ----------------

# initialize 
t = Ti
i = 1
nSteps = int(Tf/Ts+1)
t_all          = np.zeros(nSteps)
states_all     = np.zeros([nSteps, len(state), nVeh])
cmds_all       = np.zeros([nSteps, len(cmd), nVeh])
targets_all    = np.zeros([nSteps, len(targets), nVeh])
obstacles_all  = np.zeros([nSteps, len(obstacles), nObs])


# store first steps
t_all[0]                = Ti
states_all[0,:,:]       = state
cmds_all[0,:,:]         = cmd
targets_all[0,:,:]      = targets
obstacles_all[0,:,:]    = obstacles

# run
while round(t,3) < Tf:
  
    # move the target
    targets[0,:] = targets[0,:] + 0.001
    targets[1,:] = targets[1,:] + 0.002
    targets[2,:] = targets[2,:] + 0.0005


    # evolve the states (note: need to compute heading here at some point)
    state = node.evolve(Ts, state, cmd)
    
    # store
    t_all[i]                = t
    states_all[i,:,:]       = state
    cmds_all[i,:,:]         = cmd
    targets_all[i,:,:]      = targets
    obstacles_all[i,:,:]    = obstacles
    
    #increment 
    t += Ts
    i += 1
    
    # command for the next time step (target)
    #cmd, error = flock.controller(Ts, i, state,cmd, nVeh, targets, error)
    
    # flocking part
    states_q = state[0:3,:]
    states_p = state[3:6,:]
    d = 1
    r = 1.2*d
    d_prime = 0.6*d
    r_prime = 1.2*d_prime 
   
    cmd = flock.commands(states_q, states_p, obstacles, r, d, r_prime, d_prime, targets[0:3,:], targets[3:6,:])
    
    
    
#%% plot

ani = animation.animateMe(Ts, t_all, states_all, cmds_all, targets_all[:,0:3,:], obstacles_all)
#plt.show()    


         