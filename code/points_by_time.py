import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Pit:
    def __init__(self, points):
        self.points = points
        self.center = np.array([(self.points[:,0].max()+self.points[:,0].min())/2, (self.points[:,1].max()+self.points[:,1].min())/2])
        self.points_centered = self.points - self.center
    
    def findlitpoints(self, direction):
        line = direction
        line[2] = 0
        line = line.reshape(-1,1)
        # import pdb; pdb.set_trace()
        listofindices = np.where((line.T[:,0:2] @ self.points_centered.T)[0] > 0)[0]
        return listofindices

    def display(self, direction, listofindices):
        pass

def colour_box(img, xy, spacing):
    for x, y in xy:
        x = int(x)
        y = int(y)
        img[x*spacing:(x+1)*spacing, y*spacing:(y+1)*spacing] = [255, 0, 0]

illumination = loadmat("data/moon_rel_positions.mat")
# 'U_earth_point_me', 'U_me_point_me', 'U_sun_point_me', 'ets', 'ets_utc'

times = illumination['ets_utc']
directions = illumination['U_sun_point_me']

map_data = np.genfromtxt('data/mv5_M1121075381R-L.csv', delimiter=',')
waypoints = np.genfromtxt('data/trial_way_points.csv', delimiter=',')

map_data_image = map_data - map_data.min()
map_data_image = map_data_image/map_data_image.max()

map_data_image[waypoints[:,0].astype(int), waypoints[:,1].astype(int)] = 1
map_data_image = np.dstack((map_data_image,map_data_image,map_data_image))
# plt.imshow(map_data_image)
# plt.show()

pit = Pit(waypoints)

R_m90 = np.array([[0,-1,0],[1,0,0],[0,0,1]])
directions_m90 = R_m90 @ directions

mask = np.zeros((waypoints.shape[0], directions_m90.shape[1]))
for i in range(directions_m90.shape[1]):
    listoflitpoints = pit.findlitpoints(directions_m90[:,i])
    mask[listoflitpoints,i] = 1

mask = np.hstack((mask, np.zeros((mask.shape[0],1))))
illuminationtimeleft = -1*np.ones_like(mask[:,:-1])
for i in range(mask.shape[1]-1):
    litindices = np.where(mask[:,i]==1)[0]
    row_index, column_index = np.where(mask[litindices,i:]==0)
    _, first_occurance = np.unique(row_index, return_index=True)
    values = column_index[first_occurance]
    illuminationtimeleft[litindices, i] = values-1

np.save("data/litwaypointstimeleft.npy", illuminationtimeleft)
fig = plt.figure()

print("Creating Animation")
ims=[]

for i in range(illuminationtimeleft.shape[1]):
    if(i%4 ==0):
        wp_indices = np.where(illuminationtimeleft[:,i]!=-1)[0]
        # wp_indices = np.where(mask[:,i]==1)[0]
        img = np.zeros(((int(max(waypoints[:,0]))+3)*20, int((max(waypoints[:,1])+3))*20), dtype=int)
        img = np.stack((img, img, img), axis=-1)
        img_lit = img
        colour_box(img_lit, waypoints[wp_indices,:], 20)
        im = plt.imshow(img_lit, animated=True)
        ims.append([im])
        # plt.show()
    
ani = animation.ArtistAnimation(fig, ims, interval=50, blit=True,
                                repeat_delay=1000)

ani.save('data/visualize_lit.mp4')
