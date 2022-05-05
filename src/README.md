Installation
sudo apt-get install ros-melodic-jackal-desktop ros-melodic-jackal-navigation
git clone this repo


How to launch simulation
1. Change the world name in (# of jackal, e.g. six)_jackal_rtab.launch to the world you want to use
2. Change the inital position of jackals, TL, BR and MID initial positions are listed in six_jackal_rtab.launch in jackal_gazebo_rtab
3. then     roslaunch jackal_gazebo_rtab (# of jackal, e.g. six)_jackal_rtab.launch 
4. Go to baselink_calibration_(envsize).yaml in jackal_gazebo_rtab folder, wait for gazebo and rviz to respond and find the correct starting position, copy it to terminal.
5. roslaunch jackal_gazebo_rtab (# of jackal)_jackal_map_merge_global.launch
6. roslaunch jackal_gazbeo_rtab jackal_waypoint_(#number of jackals e.g. 6).launch
