#!/bin/sh
if [ "$#" -ne 1 ]; then
  echo "Incorrect arguments entered.\n\nArguments: \n\t<Illumination_index>: Integer (Options: 1,2,3)\n"
  exit 1
fi

cmake src/
make

# Running the global planner to identify the pit edges
./Global_Planner lander_to_pit

python3 code/points_by_time.py $1

./Global_Planner around_pit

python3 Python_viz/display_global_plan.py

python3 code/visualize.py