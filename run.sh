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

python3 Python_Viz/display_global_plan.py

# python3 code/visualize.py

# Move to correct folder


str="pattern"$1 
echo $str
touch hello.txt
# cp hello.txt $str"hello.txt"
cp $str"/waypoints.csv" $str"/pit_edges.csv" /home/catkin_ws/src/visualisation data
cp $str"/waypoints.csv" $str"/path_location_mapping.csv"  $str"/lander_to_pit_trajectory.csv" /home/catkin_ws/src/visualisation data  #see where local planner takes data from
