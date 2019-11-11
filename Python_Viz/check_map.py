#!/usr/bin/env python

from PIL import Image
import sys
import imageio
import cv2
import numpy as np
import csv
from numpy import genfromtxt
import ipdb
import matplotlib.pyplot as plt

def create_map_using_tif(tif_file_name):
    file_name = '/Users/harsh/Desktop/CMU_Sem_3/MRSD Project II/Real_Project_Work/Extra/mv5_M1121075381R-L.tif'
    im = imageio.imread(file_name)
    im = np.array(im)
    np.save('saved_map.npy', im)    

def load_map(map_file_name):
    im = np.load(map_file_name)
    print(im.shape)
    return im

def get_pit_edges(pit,threshold,row_low,row_high,col_low,col_high):
    border = []
    for i in range(row_low,row_high+1):
        for j in range(col_low,col_high+1):
            if(abs(abs(pit[i][j])-abs(pit[i][j+1]))>threshold or abs(abs(pit[i][j])-abs(pit[i+1][j]))>threshold or abs(abs(pit[i][j])-abs(pit[i][j-1]))>threshold or abs(abs(pit[i][j])-abs(pit[i-1][j]))>threshold):
                border.append((i,j))
    return border

def get_pit_bbox(row_low,row_high,col_low,col_high,im):
    return im[row_low:row_high,col_low:col_high]


def main():
    ### ELEVATION MAP FILE ### 
    csv_name = 'data/elevation_global_map.csv'
    my_map = genfromtxt(csv_name, delimiter=',')

    ### MAP IMAGE FILE ### 
    image_location = "data/globalmap.png"

    ### WAYPOINT/PIT EDGE FILE ### 
    waypoints_from_algo_csv_name = 'data/waypoints.csv'
    waypoints = genfromtxt(waypoints_from_algo_csv_name, delimiter=',',dtype=int)

     ### TRAJECTORY FILE ### 
    trajectory_csv_name = 'data/lander_to_pit_trajectory.csv'
    trajectory = genfromtxt(trajectory_csv_name, delimiter=',',dtype=int)   

    img = cv2.imread(image_location,0)
    # im_name = '../data/Pit_Image_Global.jpeg'
    #create_map_using_tif(file_name)
    # im = load_map(file_name)
    # plt.imshow(my_map)
    # plt.gray()
    # plt.show()
    # cv2.imwrite('color_img.jpg', im)
    row_low = 112
    row_high = 148
    col_low = 110
    col_high = 145
    pit_b_box = get_pit_bbox(row_low,row_high,col_low,col_high,my_map)
    pit = my_map
    threshold = 20
    border = get_pit_edges(pit,threshold,row_low,row_high,col_low,col_high)

    three_channel_image = np.repeat(img[:, :, np.newaxis], 3, axis=2)
    three_channel_image_copy = three_channel_image

    ### MARKING PIT_EDGE ###
    # for x,y in border:
    #     cv2.circle(three_channel_image,(y, x), 1, (0,0,255), -1)
    # cv2.imshow("pit_edges_only", three_channel_image);

    for x,y in waypoints:
        cv2.circle(three_channel_image_copy,(y, x), 1, (255,0,0), -1)

    for x,y in trajectory:
        cv2.circle(three_channel_image_copy,(y, x), 1, (0,0,255), -1)   
          
    cv2.imshow("algo_pit_edges_only", three_channel_image_copy);



    # way_points = []
    # ### GET WAYPOINTS ###  
    # with open('waypoints.csv') as csvfile:
    #     readCSV = csv.reader(csvfile, delimiter=',')
    #     for row in readCSV:
    #         x = int(row[0])
    #         y = int(row[1])
    #         way_points.append((x,y))

    # ### CREATE WAYPOINTS ###  
    # for x,y in way_points:
    #     cv2.circle(im,(y, x), 1, (0,0,255), -1)

    # path = []
    # ### GET WAYPOINTS ###  
    # with open('trajectory.csv') as csvfile:
    #     readCSV = csv.reader(csvfile, delimiter=',')
    #     for row in readCSV:
    #         x = int(row[0])
    #         y = int(row[1])
    #         path.append((x,y))

    # ### CREATE WAYPOINTS ###  
    # for x,y in path:
    #     cv2.circle(im,(y, x), 1, (0,0,255), -1)

    # cv2.imshow("with_Trajectory", im);

    '''

    Pit coordinate: [(580,520),(580,780),(800,780),(800,520)]
    '''
    cv2.waitKey();

if __name__ == '__main__':
    main()

