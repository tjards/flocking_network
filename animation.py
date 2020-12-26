#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 22 13:55:02 2020

@author: tjards
"""

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
import numpy as np
plt.rcParams['animation.ffmpeg_path'] = '/usr/local/bin/ffmpeg' #my add - this path needs to be added
Writer = animation.writers['ffmpeg']
writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

numFrames = 8 # frame rate (bigger = slower)
tail = 8

def animateMe(Ts, t_all, states_all, cmds_all, targets_all, obstacles_all):
    
    # pull out positions
    nVeh = states_all.shape[2]
    nObs = obstacles_all.shape[2]
    
    x = states_all[:,0,:]
    y = states_all[:,1,:]
    z = states_all[:,2,:]
    x_from0 = x
    y_from0 = y
    z_from0 = z
    x_v = states_all[:,3,:]
    y_v = states_all[:,4,:]
    z_v = states_all[:,5,:]
    head = 0.2
    x_head = states_all[:,0,:] + head*x_v
    y_head = states_all[:,1,:] + head*x_v
    z_head = states_all[:,2,:] + head*x_v
    x_t = targets_all[:,0,:]
    y_t = targets_all[:,1,:]
    z_t = targets_all[:,2,:]
    x_o = obstacles_all[:,0,:]
    y_o = obstacles_all[:,1,:]
    z_o = obstacles_all[:,2,:]
    r_o = obstacles_all[:,3,:]
        
    # initialize plot
    fig = plt.figure()
    ax = p3.Axes3D(fig)
    
    # axis properties 
    margins = 0.5
    maxRange = 0.5*np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() + margins
    mid_x = 0.5*(x.max()+x.min())
    mid_y = 0.5*(y.max()+y.min())
    mid_z = 0.5*(z.max()+z.min())
    ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
    ax.set_xlabel('x-direction')
    ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
    ax.set_ylabel('y-direction')
    ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
    ax.set_zlabel('Altitude')
    
    #labels
    titleTime = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)
    titleType1 = ax.text2D(0.95, 0.95, 'Title1', transform=ax.transAxes, horizontalalignment='right')
    titleType2 = ax.text2D(0.95, 0.91, 'Title2', transform=ax.transAxes, horizontalalignment='right') 
    
    # plot things that never move (targets, for now)
    #ax.scatter(targets[0,:], targets[1,:], targets[2,:], color='red', alpha=1, marker = 'o', s = 25)
    
    # initial lines 
    #line1, = ax.plot([], [], [], lw=2, color='red')
    lines_dots = []
    lines_tails = []
    lines_heads = []
    lines_targets = []
    lines_obstacles = []
    
    for i in range (0, nVeh):
        
        line_dot = ax.plot([], [], [], 'bo')
        lines_dots.extend(line_dot)
        line_tail = ax.plot([], [], [], ':', lw=1, color=[0.5,0.5,0.5])
        lines_tails.extend(line_tail)
        line_head = ax.plot([], [], [], '-', lw=1, color='black')
        lines_heads.extend(line_head)
        line_target = ax.plot([], [], [], 'go')
        lines_targets.extend(line_target)

    for j in range (0, nObs):
        
        line_obstacle = ax.plot([], [], [], 'ro', ms = 10*r_o[0,j] )
        lines_obstacles.extend(line_obstacle)
    
    def update(i):
             
        time = t_all[i*numFrames]
        x = states_all[i*numFrames,0,:]
        y = states_all[i*numFrames,1,:]
        z = states_all[i*numFrames,2,:]
        #x_from0 = states_all[0:i*numFrames,0]
        #y_from0 = states_all[0:i*numFrames,1]
        #z_from0 = states_all[0:i*numFrames,2]
        x_from0 = states_all[i*numFrames-tail:i*numFrames,0,:]
        y_from0 = states_all[i*numFrames-tail:i*numFrames,1,:]
        z_from0 = states_all[i*numFrames-tail:i*numFrames,2,:]
        x_v = states_all[i*numFrames,3,:]
        y_v = states_all[i*numFrames,4,:]
        z_v = states_all[i*numFrames,5,:]
        #norma = np.maximum(np.linalg.norm([x+x_v,y+y_v,z+z_v]),0.001)
        norma = np.maximum(np.sqrt(x_v**2 + y_v**2 + z_v**2),0.0001)
        x_head = x + head*x_v/norma
        y_head = y + head*y_v/norma
        z_head = z + head*z_v/norma
        x_point = np.vstack((x,x_head))
        y_point = np.vstack((y,y_head))
        z_point = np.vstack((z,z_head))
        x_t = targets_all[i*numFrames,0,:]
        y_t = targets_all[i*numFrames,1,:]
        z_t = targets_all[i*numFrames,2,:]
        x_o = obstacles_all[i*numFrames,0,:]
        y_o = obstacles_all[i*numFrames,1,:]
        z_o = obstacles_all[i*numFrames,2,:]
        r_o = obstacles_all[i*numFrames,3,:]
        
        
        for j in range (0, nVeh):
            
            temp1 = lines_dots[j]
            temp2 = lines_tails[j]
            temp3 = lines_heads[j]
            temp4 = lines_targets[j]
            
            temp1.set_data(x[j], y[j])
            temp1.set_3d_properties(z[j])
    
            temp2.set_data(x_from0[:,j], y_from0[:,j])
            temp2.set_3d_properties(z_from0[:,j])
            
            temp3.set_data(x_point[:,j],y_point[:,j])
            temp3.set_3d_properties(z_point[:,j])
            
            temp4.set_data(x_t[j], y_t[j])
            temp4.set_3d_properties(z_t[j])
            
        for k in range (0, nObs):
            
            temp5 = lines_obstacles[k]
            
            temp5.set_data(x_o[k], y_o[k])
            temp5.set_3d_properties(z_o[k])

        #line2.set_data(x, y)
        #line2.set_3d_properties(z)
        #line3.set_data(x_from0, y_from0)
        #line3.set_3d_properties(z_from0)
        titleTime.set_text(u"Time = {:.2f} s".format(time))
        
        return lines_dots, lines_tails, titleTime, lines_targets, lines_obstacles
    
    
    line_ani = animation.FuncAnimation(fig, update, blit=False, frames=len(t_all[0:-2:numFrames]), interval=(Ts*1000*numFrames))
    line_ani.save('Figs/animation.gif', writer=writer)
    plt.show()
    return line_ani
    
    #ax.scatter(states_all[:,0], states_all[:,1], states_all[:,2], color='blue', alpha=1, marker = 'o', s = 25) 
    
    print('animated')