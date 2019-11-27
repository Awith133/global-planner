#!/usr/bin/env python

from PIL import Image
import sys
# import imageio
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import numpy as np
import csv
from numpy import genfromtxt
# import ipdb
import matplotlib.pyplot as plt

def main():
    ### MAP IMAGE FILE ### 
    image_location = "data/globalmap.png"

    ### WAYPOINT/PIT EDGE FILE ### 
    waypoints_from_algo_csv_name = 'data/waypoints.csv'
    waypoints = genfromtxt(waypoints_from_algo_csv_name, delimiter=',',dtype=int)

     ### TRAJECTORY FILE ### 
    trajectory_csv_name = 'data/lander_to_pit_trajectory.csv'
    trajectory = genfromtxt(trajectory_csv_name, delimiter=',',dtype=int)   

    img = cv2.imread(image_location,0)

    three_channel_image = np.repeat(img[:, :, np.newaxis], 3, axis=2)
    three_channel_image_copy = three_channel_image

    for x,y in waypoints:
        cv2.circle(three_channel_image_copy,(y, x), 1, (255,0,0), -1)

    for x,y in trajectory:
        cv2.circle(three_channel_image_copy,(y, x), 1, (0,0,255), -1)   
          
    cv2.imshow("algo_pit_edges_only", three_channel_image_copy)

    ### MAP IMAGE FILE ### 
    global_plan_location = "data/global_plan.png"
    cv2.imwrite(global_plan_location, three_channel_image_copy)
    print("Global plan image saved!")

if __name__ == '__main__':
    main()

