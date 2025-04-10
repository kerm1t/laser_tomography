# -*- coding: utf-8 -*-
"""
Created on Wed Apr  9 08:51:54 2025

@author: anton
https://stackoverflow.com/questions/63984168/how-to-draw-a-circle-with-matplotlib-pyplot

"""

import numpy as np
import matplotlib.pyplot as plt

from skimage.transform import warp
from skimage.transform import rescale

# (1) draw object, e.g. circle
theta = np.linspace(0, 2*np.pi, 100)
r = 6
x = r*np.cos(theta)
y = r*np.sin(theta)

   
#plt.plot(x,y)
#plt.show()

# (2) discretize object
#l = np.zeros([50,50])
#for i in range(100):
#    l[int(r*2+x[i]),int(r*2+y[i])]=1
    
# (3) line of sight LOS, separate loops
#sum = 0
#for y in range(50):
#    for x in range(50):
#        if dimg[y,x]==1:
#            dimg[y,x]=2
#            sum = sum+1
#            break

#rng = np.zeros(50) # lidar range resolution
#for x in range(50):
#    for y in range(50):
#        if dimg[y,x]==2:
#            rng[x] = rng[x]+1
#return rng

# LOS 1 loop
#rngx = np.zeros(50) # lidar range resolution
#rngy = np.zeros(50) # lidar range resolution
#for y in range(50):
#    for x in range(50):
#        if l[y,x]==1:
#            if rngy[y] == 0:
#                rngy[y] = rngy[y]+1
#            if rngx[x] == 0:
#                l[y,x]=2
#                rngx[x] = rngx[x]+1


####################
# Now with rotation!
####################

#dimg = np.zeros([50,50])
#for i in range(100):
#    dimg[int(r*2+x[i]),int(r*2+y[i])]=1


# (1) draw object, e.g. box
dimg = np.zeros((50, 50))
dimg[5, 2:20] = 1  # Top edge
dimg[5:10, 2] = 1  # Bottom edge
dimg[10, 2:20] = 1  # Left edge
dimg[5:10, 20] = 1  # Right edge

dimg = np.zeros((50, 50))
dimg[15, 20:30] = 1  # Top edge
dimg[15:20, 20] = 1  # Bottom edge
dimg[20, 20:30] = 1  # Left edge
dimg[15:20, 30] = 1  # Right edge

# LOS, einzeln
def los(dimg): # depthimage
    sum = 0
    for y in range(50):
        for x in range(50):
            if dimg[y,x]>0:
                dimg[y,x]=2
                sum = sum+1
                break
    
    rng = np.zeros(50) # lidar range resolution
    for x in range(50):
        for y in range(50):
            if dimg[y,x]==2:
                rng[x] = rng[x]+1
    return rng

rng = los(dimg)


def laser_trafo(padded_image, center, theta):
    radon_image = np.zeros([50, len(theta)])
    for angle in enumerate(np.deg2rad(theta)):
        cos_a, sin_a = np.cos(angle), np.sin(angle)
        R = np.array(
            [
                [cos_a, sin_a, -center * (cos_a + sin_a - 1)],
                [-sin_a, cos_a, -center * (cos_a - sin_a - 1)],
                [0, 0, 1],
            ]
        )
        rotated = warp(padded_image, R, clip=False)
    #    radon_image[:, i] = rotated.sum(0)
#    los    

#dtheta = 50
theta = np.linspace(0.0, 360.0, 360, endpoint=False)

padded_image = dimg
#padded_image = rescale(dimg, scale=0.4, mode='reflect', channel_axis=None)

#laser_trafo(dimg, 25, theta)
center = 25
#for angle in enumerate(np.deg2rad(theta)):
#    cos_a, sin_a = np.cos(angle), np.sin(angle)
rng = np.zeros([50,360]) # lidar range resolution
th = np.deg2rad(theta)
for i in range(1,360):
    print(th[i])
    cos_a = np.cos(th[i])
    sin_a = np.sin(th[i])
    R = np.array(
        [
            [cos_a, sin_a, -center * (cos_a + sin_a - 1)],
            [-sin_a, cos_a, -center * (cos_a - sin_a - 1)],
            [0, 0, 1],
        ]
    )
    rotated = warp(padded_image, R, clip=False)
#    plt.plot(rotated)
#    plt.show()
    rng[:,i] = los(rotated)

#plt.plot(rng)    
plt.imshow(rng)
plt.show()


from skimage.transform import iradon

reconstruction_fbp = iradon(rng, theta=theta, filter_name='ramp')
error = reconstruction_fbp - dimg
print(f'FBP rms reconstruction error: {np.sqrt(np.mean(error**2)):.3g}')

imkwargs = dict(vmin=-0.2, vmax=0.2)
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 4.5), sharex=True, sharey=True)
ax1.set_title("Reconstruction\nFBP")
ax1.imshow(reconstruction_fbp, cmap=plt.cm.Greys_r)
ax2.set_title("Reconstruction error\nFBP")
ax2.imshow(reconstruction_fbp - dimg, cmap=plt.cm.Greys_r, **imkwargs)
plt.show()