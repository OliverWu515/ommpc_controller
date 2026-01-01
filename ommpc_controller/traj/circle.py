#--------------------------------------
#Generate reference trajectory
#--------------------------------------

import numpy as np
import math

# Parameters
sample_time = 0.01             # seconds
duration = 40                  # seconds

r = 1.2
T = 8
v = 2 * r * 3.1415926 / T

x0 = r                     
y0 = 0
z0 = 0.8

clockwise = True
factor = -1 if clockwise else 1

# trajectory
traj = np.zeros((int(duration/sample_time+1),8))
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

traj[:,0] = -r*np.cos(t*v/r)+x0
traj[:,1] = factor*-r*np.sin(t*v/r)+y0
traj[:,2] = z0
traj[:,3] = v*np.sin(t*v/r)
traj[:,4] = factor*-v*np.cos(t*v/r)
traj[:,5] =  0
traj[:,6] =  0
traj[:,7] =  0


# write to txt
np.savetxt('circle.txt',traj,fmt='%f')
