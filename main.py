#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The project implements 3_D flocking on a double-integrator model, as
described in:
    
Olfati-Saber, R, "Flocking for Multi-Agent Dynamic Systems:
Algorithms and Theory", IEEE TRANSACTIONS ON AUTOMATIC CONTROL, 
Vol. 51 (3), 3 Mar 2006

Created on Tue Dec 22 11:48:18 2020

@author: tjards

"""


#%% Import stuff
# --------------

#from scipy.integrate import ode
import numpy as np
import animation 
import ctrl_flock as flock
import dynamics_node as node
import flock_tools as flock_tools

#%% Setup Simulation
# ------------------
Ti = 0          # initial time
Tf = 5          # final time 
Ts = 0.02       # sample time
nVeh = 10       # number of vehicles
iSpread = 10    # initial spread of vehicles 

# Vehicles states
# ---------------
state = np.zeros((6,nVeh))
state[0,:] = iSpread*(np.random.rand(1,nVeh)-0.5)               # position (x)
state[1,:] = iSpread*(np.random.rand(1,nVeh)-0.5)               # position (y)
state[2,:] = np.maximum((iSpread*np.random.rand(1,nVeh)+1.5),0) # position (z)
state[3,:] = 0                                                  # velocity (vx)
state[4,:] = 0                                                  # velocity (vy)
state[5,:] = 0                                                  # velocity (vz)

# Commands
# --------
cmd = np.zeros((3,nVeh))
cmd[0] = np.random.rand(1,nVeh)-0.5      # command (x)
cmd[1] = np.random.rand(1,nVeh)-0.5      # command (y)
cmd[2] = np.random.rand(1,nVeh)-0.5      # command (z)

# Targets
# -------
targets = 4*(np.random.rand(6,nVeh)-0.5)
targets[0,:] = 0 #5*(np.random.rand(1,nVeh)-0.5)
targets[1,:] = 0 #5*(np.random.rand(1,nVeh)-0.5)
targets[2,:] = 1
targets[3,:] = 0
targets[4,:] = 0
targets[5,:] = 0
error = state[0:3,:] - targets[0:3,:]

#%% Define obstacles
# ------------------
nObs = 5    # number of obstacles
obstacles = np.zeros((4,nObs))

#manual (comment out if random)
# obstacles[0,:] = 0    # position (x)
# obstacles[1,:] = 0    # position (y)
# obstacles[2,:] = 0    # position (z)
# obstacles[3,:] = 0

#random (comment this out if manual)
obstacles[0,:] = iSpread*(np.random.rand(1,nObs)-0.5)                   # position (x)
obstacles[1,:] = iSpread*(np.random.rand(1,nObs)-0.5)                   # position (y)
obstacles[2,:] = np.maximum(iSpread*(np.random.rand(1,nObs)-0.5),2)     # position (z)
obstacles[3,:] = np.random.rand(1,nObs)+0.5                             # radii of obstacle(s)

# Walls/Floors 
# - these are defined manually as planes
# --------------------------------------   
nWalls = 1
walls = np.zeros((6,nWalls)) 
walls_plots = np.zeros((4,nWalls))

# add the ground at z = 0:
newWall0, newWall_plots0 = flock_tools.buildWall('horizontal', 0) 

# load the ground into constraints   
walls[:,0] = newWall0[:,0]
walls_plots[:,0] = newWall_plots0[:,0]

# add other planes (comment out by default)

# newWall1, newWall_plots1 = flock_tools.buildWall('diagonal1a', 3) 
# newWall2, newWall_plots2 = flock_tools.buildWall('diagonal1b', -3) 
# newWall3, newWall_plots3 = flock_tools.buildWall('diagonal2a', -3) 
# newWall4, newWall_plots4 = flock_tools.buildWall('diagonal2b', 3)

# load other planes (comment out by default)

# walls[:,1] = newWall1[:,0]
# walls_plots[:,1] = newWall_plots1[:,0]
# walls[:,2] = newWall2[:,0]
# walls_plots[:,2] = newWall_plots2[:,0]
# walls[:,3] = newWall3[:,0]
# walls_plots[:,3] = newWall_plots3[:,0]
# walls[:,4] = newWall4[:,0]
# walls_plots[:,4] = newWall_plots4[:,0]

#%% Run Simulation
# ----------------------
t = Ti
i = 1
nSteps = int(Tf/Ts+1)
t_all          = np.zeros(nSteps)
states_all     = np.zeros([nSteps, len(state), nVeh])
cmds_all       = np.zeros([nSteps, len(cmd), nVeh])
targets_all    = np.zeros([nSteps, len(targets), nVeh])
obstacles_all  = np.zeros([nSteps, len(obstacles), nObs])
t_all[0]                = Ti
states_all[0,:,:]       = state
cmds_all[0,:,:]         = cmd
targets_all[0,:,:]      = targets
obstacles_all[0,:,:]    = obstacles

while round(t,3) < Tf:
  
    # Evolve the target
    # -----------------
    tSpeed = 0
    targets[0,:] = targets[0,:] + tSpeed*0.002
    targets[1,:] = targets[1,:] + tSpeed*0.005
    targets[2,:] = targets[2,:] + tSpeed*0.0005


    # Evolve the states
    # -----------------
    state = node.evolve(Ts, state, cmd)
    
    # Store results
    # -------------
    t_all[i]                = t
    states_all[i,:,:]       = state
    cmds_all[i,:,:]         = cmd
    targets_all[i,:,:]      = targets
    obstacles_all[i,:,:]    = obstacles
    
    # Increment 
    # ---------
    t += Ts
    i += 1
        
    # Compute commands (next step)
    # ----------------------------
    states_q = state[0:3,:]     # positions
    states_p = state[3:6,:]     # velocities 
    d = 2                       # lattice scale (distance between a-agents)
    r = 1.2*d                   # interaction range of a-agents
    d_prime = 0.6*d             # distance between a- and b-agents
    r_prime = 1.2*d_prime       # interaction range of a- and b-agents
    
    cmd = flock.commands(states_q, states_p, obstacles, walls, r, d, r_prime, d_prime, targets[0:3,:], targets[3:6,:])
        
#%% Produce animation of simulation
# ---------------------------------
showObs = 1 # (0 = don't show obstacles, 1 = show obstacles, 2 = show obstacles + floors/walls)
ani = animation.animateMe(Ts, t_all, states_all, cmds_all, targets_all[:,0:3,:], obstacles_all, r, d, walls_plots, showObs)
#plt.show()    




