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
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation

#%% Setup 

# Simulation Parameters
# ----------------
Ti = 0      # initial time
Tf = 5      # final time 
Ts = 0.1    # sample time

state = np.zeros(6)
state[0] = 0    # position (x)
state[1] = 0    # position (y)
state[2] = 0    # position (z)
state[3] = 0    # velocity (vx)
state[4] = 0    # velocity (vy)
state[5] = 0    # velocity (vz)
cmd = np.zeros(3)
cmd[0] = 0.1      # command (x)
cmd[1] = 0.3      # command (y)
cmd[2] = 0.2      # command (z)


# Define dynamics
# ---------------
def state_dot(t, state, cmd):
    
    dynDot = np.array([
        [state[3]],
        [state[4]],
        [state[5]],
        [cmd[0]],
        [cmd[1]],
        [cmd[2]]])
    
    dstate = np.zeros(6)
    dstate[0] = dynDot[0]
    dstate[1] = dynDot[1]
    dstate[2] = dynDot[2]
    dstate[3] = dynDot[3]
    dstate[4] = dynDot[4]
    dstate[5] = dynDot[5]
    
    return dstate

# Set integrator
# -------------
integrator = ode(state_dot).set_integrator('dopri5', first_step='0.00005', atol='10e-6', rtol='10e-6')
integrator.set_initial_value(state, Ti)

#%% Run simulation 

# initialize 
t = Ti
i = 1
nSteps = int(Tf/Ts+1)
t_all          = np.zeros(nSteps)
states_all     = np.zeros([nSteps, len(state)])
cmds_all       = np.zeros([nSteps, len(cmd)])

# store first steps
t_all[0]              = Ti
states_all[0,:]       = state
cmds_all[0,:]         = cmd

# run
while round(t,3) < Tf:
    
    # integrate through dynamics 
    integrator.set_f_params(cmd)
    state = integrator.integrate(t, t+Ts)
    t += Ts
    
    # store
    # store first steps
    t_all[i]              = Ti
    states_all[i,:]       = state
    cmds_all[i,:]         = cmd
    i += 1
    
#%% plot
fig = plt.figure()
ax = p3.Axes3D(fig)   
ax.scatter(states_all[:,0], states_all[:,1], states_all[:,2], color='blue', alpha=1, marker = 'o', s = 25)    

    


         