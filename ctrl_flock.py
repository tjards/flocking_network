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
    kp = 0.03
    kd = 0.4
    
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



# Flocking equations
# ref: Reza Olfati-Saber,"Flocking for Multi-Agent Dynamic Systems:
# Algorithms and Theory", IEEE TRANSACTIONS ON AUTOMATIC CONTROL, 
# Vol. 51 (3), 3 Mar 2006
# ================================================================

# hyper parameters 
#r = 5       # interaction range
a = 0.5
b = 0.5
c = np.divide(np.abs(a-b),np.sqrt(4*a*b)) # note: (0 < a <= b, c = abs(a-b)/sqrt(4ab))
eps = 0.1
h = 0.9
pi = 3.141592653589793
c1_a = 2
c2_a = 2*np.sqrt(2)



# High-level equations
# --------------------
def flock_sum(u_int, u_obs, u_nav):    
    u_sum = u_int + u_obs + u_nav
    return u_sum

# ~~ common functions ~~ 

def sigma_norm(z):    
    norm_sig = (1/eps)*(np.sqrt(1+eps*np.linalg.norm(z)**2)-1)
    return norm_sig
    
def n_ij(q_i, q_j):
    n_ij = np.divide(q_j-q_i,np.sqrt(1+eps*np.linalg.norm(q_j-q_i)**2))    
    return n_ij

def sigma_1(z3):    
    sigma_1 = np.divide(z3,np.sqrt(1+z3**2))    
    return sigma_1

def rho_h(z4):    
    if 0 <= z4 < h:
        rho_h = 1        
    elif h <= z4 < 1:
        rho_h = 0.5*(1+np.cos(pi*np.divide(z4-h,1-h)))    
    else:
        rho_h = 0  
    return rho_h



    
# Flocking Equations 
# -----------------------------------
def commands(states_q, states_p, obstacles, r, d, r_prime, d_prime, targets):   
    r_a = sigma_norm(r)
    d_a = sigma_norm(d)
    r_b = sigma_norm(r_prime)
    d_b = sigma_norm(d_prime)     
    #initialize the terms for each node
    u_int = np.zeros((3,states_q.shape[1]))     # interactions
    u_obs = np.zeros((3,states_q.shape[1]))     # obstacles 
    
    # for each vehicle/node in the network
    for k_node in range(states_q.shape[1]): 
        
    # Interaction Equations (phi_alpha)
    # --------------------------------            
        # search through each neighbour
        for k_neigh in range(states_q.shape[1]):
            # except for itself (duh):
            if k_node != k_neigh:
                # compute the euc distance between them
                dist = np.linalg.norm(states_q[:,k_node]-states_q[:,k_neigh])
                # if it is within the interaction range
                if dist < r:
                    # compute the interaction command
                    u_int[:,k_node] += c1_a*phi_a(states_q[:,k_node],states_q[:,k_neigh],r_a, d_a)*n_ij(states_q[:,k_node],states_q[:,k_neigh]) + a_ij(states_q[:,k_node],states_q[:,k_neigh],r_a)*(states_p[:,k_neigh]-states_p[:,k_node]) 
                               
    
    # Obstacle Avoidance (phi_beta)
    # -----------------------------   
        #search through each obstacle 
        for k_obstacle in range(obstacles.shape[1]):
    
            u_obs = u_obs
            
    
    cmd = u_int + u_obs
    
    return cmd
                    
    
    
    


# ~~ the phi_alpha group ~~ 
 
def phi_a(q_i, q_j, r_a, d_a): 
    #d = np.linalg.norm(q_j-q_i)
    #d_a = sigma_norm(d)
    #r_a = sigma_norm(r)
    z1 = sigma_norm(q_j-q_i)        
    phi_a = rho_h(z1/r_a) * phi(z1-d_a)    
    return phi_a
    
def phi(z2):    
    phi = 0.5*((a+b)*sigma_1(z2+c)+(a-b))    
    return phi 
        
def a_ij(q_i, q_j, r_a):        
    a_ij = rho_h(sigma_norm(q_j-q_i)/r_a)
    return a_ij

# ~~ the phi_beta group ~~

def b_ik(q_ik, q_j, d_a):        
    b_ik = rho_h(sigma_norm(q_j-q_ik)/d_a)
    return b_ik

