import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def colour_box(img, xy, spacing, color):
    for x, y in xy:
        x = int(x)
        y = int(y)
        img[x*spacing:(x+1)*spacing, y*spacing:(y+1)*spacing] += color

illuminationtimeleft = np.load("data/litwaypointstimeleft.npy")
waypoints = np.genfromtxt('data/waypoints.csv', delimiter=',')
robot_waypoints = np.genfromtxt('data/time_location_mapping.csv', delimiter=',')

print("Creating Animation")
ims=[]

fig = plt.figure()

counter = 0
r_x, r_y = 0,0

for i in range(illuminationtimeleft.shape[1]):
    
    if(i == robot_waypoints[counter,0]):
        r_x = robot_waypoints[counter,1]
        r_y = robot_waypoints[counter,2]
        counter+=1
    if(i%4 ==0):
        wp_indices = np.where(illuminationtimeleft[:,i]!=-1)[0]
        # wp_indices = np.where(mask[:,i]==1)[0]
        img = np.zeros(((int(max(waypoints[:,0]))+3)*20, int((max(waypoints[:,1])+3))*20), dtype=int)
        img = np.stack((img, img, img), axis=-1)
        img_lit = img
        colour_box(img_lit, waypoints[wp_indices,:], 20, [255,0,0])
        colour_box(img_lit, [(r_x,r_y)], 20, [0,255,0])
        img_lit = img_lit[int(min(waypoints[:,0])-3)*20:, int((min(waypoints[:,1])-3))*20:]
        im = plt.imshow(img_lit, animated=True)
        ims.append([im])
    # plt.show()
    
ani = animation.ArtistAnimation(fig, ims, interval=50, blit=True,
                                repeat_delay=1000)

ani.save('data/visualize_lit.mp4')