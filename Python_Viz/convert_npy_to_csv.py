import numpy as np
import csv
import ipdb

def convert_npy_to_csv(file_name):
	np.savetxt("lit_waypoints.csv", file_name, delimiter=",")


def inspect_npy_file(file_name):
	data = np.load(file_name)
	convert_npy_to_csv(data)
	ipdb.set_trace()


def main():
	file_name = "/Users/harsh/Desktop/CMU_Sem_3/MRSD Project II/Real_Project_Work/Create_Global_Waypoints/Python_Viz/litwaypointstimeleft_demo.npy"
	inspect_npy_file(file_name)


if __name__ == '__main__':
    main()