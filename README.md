## Project_ariac: moveArm
This module is capable of starting the competition, moving the UR10 arm to the first piston_rod_part, picking it and then moving along the 
linear actuator it can reach the tray on the agv and drop the part. After this it will follow the trajectory back to bin. So we need to find
a way for it to go pick the following piston_rod_part.

## Standard install via command-line
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ cd src/
$ git checkout -b harish--single-branch https://github.com/raviBhadeshiya/project_ariac
$ cd ..
$ catkin_make
```
To run qualifier1 type:
```
$ rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual1a.yaml ~/catkin_ws/src/project_ariac/config/qual1a_config.yaml 
```
and in new terminal type:
```
$ rosrun project_ariac project_ariac_node
```
Note:

