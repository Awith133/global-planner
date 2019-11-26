import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

def colour_box(img, xy, spacing, color, force=False):
    for x, y in xy:
        x = int(x)
        y = int(y)
        if(force):
            img[x*spacing:(x+1)*spacing, y*spacing:(y+1)*spacing] = color
        else:
            img[x*spacing:(x+1)*spacing, y*spacing:(y+1)*spacing] += color

illuminationtimeleft = np.load("data/litwaypointstimeleft.npy")
waypoints = np.genfromtxt('data/waypoints.csv', delimiter=',')
robot_waypoints = np.genfromtxt('data/time_location_mapping.csv', delimiter=',')

print("Creating Animation")
ims=[]

custom_lines = [Rectangle((0,0), 1, 1, color=[0,1,0]),
                Rectangle((0,0), 1, 1, color=[1,1,0]),
                Rectangle((0,0), 1, 1, color=[1,0,0]),
                Rectangle((0,0), 1, 1, color=[int(0.8*255)/255.0,int(0.8*255)/255.0,int(0.8*255)/255])]

fig = plt.figure(figsize=(8, 6))
ax = plt.subplot(111)
ax.legend(custom_lines, ['Planned Robot Position IS NOT at a Vantage Point', 'Planned Robot Position IS at a Vantage Point', 'Potential Vantage Points', 'Pit Edge Points'], loc='upper center', bbox_to_anchor=(0.5, 1.05), ncol=2, fancybox=True, shadow=True)
# handles, labels = ax.get_legend_handles_labels()
# lgd = ax.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5,-0.1))
plt.title('Illumination of Mare Ingenii at 33S and 163E', y=-0.1)
plt.axis('off')

counter = 0
r_x, r_y = 0,0

all_indices = np.arange(illuminationtimeleft.shape[0])
gray = int(0.8*255)
for i in range(illuminationtimeleft.shape[1]):
    
    if(i == robot_waypoints[counter,0]):
        r_x = robot_waypoints[counter,1]
        r_y = robot_waypoints[counter,2]
        counter+=1
    if(i%4 ==0):
        wp_indices = np.where(illuminationtimeleft[:,i]!=-1)[0]
        nonvantage_indices = np.array(list(set(all_indices) - set(wp_indices)))
        # wp_indices = np.where(mask[:,i]==1)[0]
        img = np.zeros(((int(max(waypoints[:,0]))+3)*20, int((max(waypoints[:,1])+3))*20), dtype=int)
        img = np.stack((img, img, img), axis=-1)
        img_lit = img
        if(len(wp_indices)!=0):
            colour_box(img_lit, waypoints[nonvantage_indices,:], 20, [gray,gray,gray])
        colour_box(img_lit, waypoints[wp_indices,:], 20, [255,0,0])
        colour_box(img_lit, [(r_x,r_y)], 20, [0,255,0], force=False)
        img_lit = img_lit[int(min(waypoints[:,0])-7)*20:, int((min(waypoints[:,1])-7))*20:]
        im = ax.imshow(img_lit, animated=True)
        ims.append([im])
    # plt.show()
    
ani = animation.ArtistAnimation(fig, ims, interval=50, blit=True,
                                repeat_delay=1000)

ani.save('data/visualize_lit.mp4')