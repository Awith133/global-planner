#!/bin/sh
if [ "$#" -ne 1 ]; then
  echo "Incorrect arguments entered.\n\nArguments: \n\t<Illumination_index>: Integer (Options: 1,2,3)\n"
  exit 1
fi

/home/hash/Downloads/cmake-3.14.0-rc3-Linux-x86_64/bin/cmake src/
make

# Running the global planner to identify the pit edges
./Global_Planner lander_to_pit

python3 code/points_by_time.py $1

./Global_Planner around_pit

python3 Python_Viz/display_global_plan.py    #Uncomment this before running on PC

# python3 code/visualize.py

# Move to correct folder

#str="pattern"$1

#mkdir -p $str; mv data/waypoints.csv  data/global_plan.png data/lander_to_pit_trajectory.csv data/path_location_mapping.csv $str
 
#Pit edges are generated externally and need to be copied manually if it is changed.
#mv $str/waypoints.csv $str/global_waypoints.csv
#cp $str"/global_waypoints.csv" $str"/pit_edges.csv" $str"/global_plan.png" /home/hash/catkin_ws/src/visualization/data
#cp $str"/global_waypoints.csv" $str"/path_location_mapping.csv"  $str"/lander_to_pit_trajectory.csv" /home/hash/catkin_ws/src
