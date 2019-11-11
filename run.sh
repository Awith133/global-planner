cmake src/
make

# Running the global planner to identify the pit edges
./Global_Planner lander_to_pit

python3 code/points_by_time.py

./Global_Planner around_pit

python3 code/visualize.py