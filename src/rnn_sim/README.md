To run simulation of two robots:
1. roslaunch jackal_gazebo_rtab two_jackal_rtab.launch
2. open base_link_calibration.yaml and select the proper robot calibration, run it in terminal
3. roslaunch jackal_gazebo_rtab jackal_map_merge_demo.launch
4. roslaunch jackal_gazebo_rtab jackal_waypoint.launch
5. within 2s, rosrun rnn_sim jackal_map_exchange.py
Note: This map exchange process is simulated by manually counting the time when two robots should perform map merge. 

To run simulation for 4 robots:
1. roslaunch jackal_gazebo_rtab four_jackal_rtab.launch
2. open base_link_calibration.yaml and select the proper robot calibration, run it in terminal
3. roslaunch jackal_gazebo_rtab four_jackal_map_merge.launch
4. roslaunch jackal_gazebo_rtab jackal_waypoint_4.launch

To run simulation for 6 robots:
1. roslaunch jackal_gazebo_rtab four_jackal_rtab.launch
2. open base_link_calibration.yaml and copy all services run them in terminal
3. roslaunch jackal_gazebo_rtab six_jackal_map_merge.launch
4. roslaunch jackal_gazebo_rtab jackal_waypoint_6.launch

To calibrate map merge node:
1. run step 1-3
2. In six_jackal.rviz choose to display jackal1/map and /map topics, find the offset between two maps and adjust the value in $(arg namespace)/map_merge starting from line 196 (please do not change the yaw value)