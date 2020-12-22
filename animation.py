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

numFrames = 4 # frame rate (bigger = slower)
tail = 8

def animateMe(Ts, t_all, states_all, cmds_all, landmarks,nVeh):
    
    # pull out positions
    x = states_all[:,0,:]
    y = states_all[:,1,:]
    z = states_all[:,2,:]

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
    
    # plot things that never move (landmarks)
    ax.scatter(landmarks[:,0], landmarks[:,1], landmarks[:,2], color='red', alpha=1, marker = 'o', s = 25)
    
    # initial lines 
    #line1, = ax.plot([], [], [], lw=2, color='red')
    line2, = ax.plot([], [], [], 'bo')
    line3, = ax.plot([], [], [], '--', lw=1, color='blue')
    
    def update(i):
             
        time = t_all[i*numFrames]
        x = states_all[i*numFrames,0]
        y = states_all[i*numFrames,1]
        z = states_all[i*numFrames,2]
        #x_from0 = states_all[0:i*numFrames,0]
        #y_from0 = states_all[0:i*numFrames,1]
        #z_from0 = states_all[0:i*numFrames,2]
        x_from0 = states_all[i*numFrames-tail:i*numFrames,0]
        y_from0 = states_all[i*numFrames-tail:i*numFrames,1]
        z_from0 = states_all[i*numFrames-tail:i*numFrames,2]
        
        line2.set_data(x, y)
        line2.set_3d_properties(z)
        line3.set_data(x_from0, y_from0)
        line3.set_3d_properties(z_from0)
        titleTime.set_text(u"Time = {:.2f} s".format(time))
        
        return line2, line3, titleTime
    
    
    line_ani = animation.FuncAnimation(fig, update, blit=False, frames=len(t_all[0:-2:numFrames]), interval=(Ts*1000*numFrames))
    #line_ani.save('Figs/animation.gif', writer=writer)
    plt.show()
    return line_ani
    
    #ax.scatter(states_all[:,0], states_all[:,1], states_all[:,2], color='blue', alpha=1, marker = 'o', s = 25) 
    
    print('animated')