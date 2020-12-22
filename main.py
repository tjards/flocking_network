#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 22 11:48:18 2020

Implement a double integrator 


@author: tjards
"""


#%% Imports
from scipy.integrate import ode
#from scipy.integrate import odeint
import numpy as np
# import matplotlib.pyplot as plt
# import mpl_toolkits.mplot3d.axes3d as p3
# from matplotlib import animation
import animation 

#%% Setup 

# Simulation Parameters
# ----------------
Ti = 0      # initial time
Tf = 5      # final time 
Ts = 0.1    # sample time
nVeh = 15
iSpread = 3

state = np.zeros((6,nVeh))
state[0,:] = iSpread*np.random.rand(1,nVeh)    # position (x)
state[1,:] = iSpread*np.random.rand(1,nVeh)    # position (y)
state[2,:] = iSpread*np.random.rand(1,nVeh)    # position (z)
state[3,:] = 0    # velocity (vx)
state[4,:] = 0    # velocity (vy)
state[5,:] = 0    # velocity (vz)
cmd = np.zeros((3,nVeh))
cmd[0] = np.random.rand(1,nVeh)-0.5      # command (x)
cmd[1] = np.random.rand(1,nVeh)-0.5      # command (y)
cmd[2] = np.random.rand(1,nVeh)-0.5      # command (z)
landmarks = np.zeros((1,3))
landmarks[0,0] = 0
landmarks[0,1] = 0
landmarks[0,2] = 0



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
integrator.set_initial_value(state[:,0], Ti)

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
    
    # integrate through dynamics
    
    
    for j in range(0,nVeh):
        
        #this isn't working .... try odeint?
    
        integrator.set_initial_value(states_all[0,:,j],Ts)
        integrator.set_f_params(cmd[:,j])
        #state[:,j] = integrator.integrate(t, t+Ts)
        state[:,j] = integrator.integrate(t, t+Ts)
        
    t += Ts
    
    # store
    # store first steps
    t_all[i]              = t
    states_all[i,:,:]       = state
    cmds_all[i,:,:]         = cmd
    i += 1
    
#%% plot

ani = animation.animateMe(Ts, t_all, states_all, cmds_all, landmarks, nVeh)
#plt.show()    


         