#!/usr/bin/env python

from PIL import Image
import sys
import imageio
import cv2
import numpy as np
import csv

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
        for j in range(col_low,col_high):
            if(pit[i][j]-pit[i][j+1]>threshold or pit[i][j]-pit[i+1][j]>threshold or pit[i][j]-pit[i][j-1]>threshold or pit[i][j]-pit[i-1][j]>threshold):
                border.append((i,j))
    return border

def get_pit_bbox(row_low,row_high,col_low,col_high,im):
    return im[row_low:row_high,col_low:col_high]


def main():
    file_name = '/Users/harsh/Desktop/CMU_Sem_3/MRSD Project II/Real_Project_Work/Extra/mv5_M1121075381R-L.tif'
    #create_map_using_tif(file_name)
    im = load_map('saved_map.npy')
    cv2.imwrite('color_img.jpg', im)
    # row=800
    # col=520
    # cv2.circle(im,(col, row), 5, (0,255,0), -1)
    # cv2.imshow("image", im);
    row_low = 580
    row_high = 800
    col_low = 520
    col_high = 780
    pit_b_box = get_pit_bbox(row_low,row_high,col_low,col_high,im)
    pit = im
    threshold = 0.1
    border = get_pit_edges(pit,threshold,row_low,row_high,col_low,col_high)

    # print("Length is: \t",len(border))
    # cv2.imshow("original_image", im);

    ### MARKING PIT_EDGE ###
    for x,y in border:
        cv2.circle(im,(y, x), 1, (0,0,255), -1)
    cv2.imshow("pit_edges_only", im);

    way_points = []
    ### GET WAYPOINTS ###  
    with open('waypoints.csv') as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
            x = int(row[0])
            y = int(row[1])
            way_points.append((x,y))

    ### CREATE WAYPOINTS ###  
    for x,y in way_points:
        cv2.circle(im,(y, x), 1, (0,0,255), -1)

    path = []
    ### GET WAYPOINTS ###  
    with open('trajectory.csv') as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
            x = int(row[0])
            y = int(row[1])
            path.append((x,y))

    ### CREATE WAYPOINTS ###  
    for x,y in path:
        cv2.circle(im,(y, x), 1, (0,0,255), -1)

    cv2.imshow("with_Trajectory", im);

    '''

    Pit coordinate: [(580,520),(580,780),(800,780),(800,520)]
    '''
    cv2.waitKey();

if __name__ == '__main__':
    main()

