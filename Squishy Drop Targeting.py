#!/usr/bin/env python
# coding: utf-8

# In[70]:


import numpy as np
import matplotlib.pylab as plt
from scipy.integrate import ode
import math
from random import random
import sys
from pycse import odelay
from mpl_toolkits import mplot3d

get_ipython().run_line_magic('matplotlib', 'inline')


# In[47]:


#### Known Data:
    
Cd = 0.5;
D = 21.5 * 0.0254; # m
A = math.pi * math.pow((D/2),2); #Area of robot in m^2
m = 1.15; # kg
rho = 1.225; #kg/m^3
g = 9.81; #m/s^2
k = (0.5) * (rho) * A * Cd;

#### Terminal velocity calculator
v_t = math.pow(((m * g)/k),0.5);  


# In[82]:


#### Initial condition:
v_wind1 = random(); # Assumption
v_wind2 = random();

xhx0 = 0;
xv0 = 30.48; #100 ft in meters
xv_dot0 = 0;
xhx_dot0 = v_wind1; #when drone is hovering during drop
xhz0 = 0;
xhz_dot0 = v_wind2;

x0 = np.array([xhx0, xhx_dot0, xhz0, xhz_dot0, xv0, xv_dot0]);


# In[83]:


#### Fall dynamics before terminal velocity
def freefall(x, t):
    #k = ((0.5) * (rho) * A * Cd)/m;
    #v_wind = rand(1);
    #print("in freefall")
    xdot0 = x[1]
    xdot1 = -k * math.pow(x[1] - v_wind1, 2);
    xdot2 = x[3]
    xdot3 = -k * math.pow(x[3] - v_wind2, 2);
    xdot4 = x[5];
    xdot5 = -g - k * math.pow(x[5], 2);
    
    
    xdot = np.array([xdot0, xdot1, xdot2, xdot3, xdot4, xdot5]);
    return xdot;


# In[84]:


#### Fall dynamics after terminal velocity
def terminalfall(x, t):
    #k = ((0.5) * (rho) * A * Cd)/m;
    #v_wind = rand(1);
    xdot1 = x[1]
    xdot2 = -k * math.pow(x[1] - v_wind1, 2);
    xdot3 = x[3];
    xdot4 = -k * math.pow(x[3] - v_wind2, 2);
    xdot5 = x[5];
    xdot6 = -g + k * math.pow(v_t, 2);
    
    xdot = np.array([xdot1, xdot2, xdot3, xdot4, xdot5, xdot6]);
    return xdot;


# In[85]:


def event_before_vt(x, t):
    #### Terminal velocity calculator
    #print("In event_before_vt")
    #v_t = math.pow(((m * g)/k),0.5);
    value = x[5] + v_t - 0.001; #Event when we reach terminal velocity
    isterminal = True
    direction = 0
    return value, isterminal, direction


# In[86]:


def event_before_ground(x, t):
    #print("In event_before_ground, x:", x)
    value = x[4] - 0.001; #Event when we reach ground
    isterminal = True
    direction = 0
    return value, isterminal, direction


# In[87]:


#### Dynamics

t0 = 0; #Initial time
tspan = np.linspace(0, 10, 100);

t, x, XE, FE, IE = odelay(freefall, x0, tspan, events = [event_before_vt])

print("Time shape:", t.shape)
print("X Shape:", x.shape)


# In[88]:


l1 = len(t)
x0 = x[l1 - 1,:]
t0 = t[l1 - 1]

t1, x1, XE, FE, IE = odelay(terminalfall, x0, tspan, events = [event_before_ground])

x = np.concatenate((x,x1), axis = 0)
t = np.concatenate((t,t1), axis = 0)


# In[89]:


ax = plt.axes(projection='3d')
ax.plot3D(x[:,0], x[:,2], x[:,4], 'gray')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
#print(x[:,0])
#plt.title("Plot Title")


# In[90]:


l2 = len(t)
print(l2)
print("Target location coordinates")
print("X coordinate", x[l2-1,0])
print("Y coordinate", x[l2-1,2])
print("Z coordinate", x[l2-1,4])


# In[ ]:




