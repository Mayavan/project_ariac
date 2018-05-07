# project_ariac
[![Packagist](https://img.shields.io/pypi/l/Django.svg)](LICENSE)

This repository is a showcase of the projects done in the course ENPM 809b at University of Maryland.

The objective of the Agile Robotics for Industrial Automation Competition (ARIAC) is to test the agility of industrial robot systems, with the goal of enabling industrial robots on the shop floors to be more productive, more autonomous, and be more responsive to the needs of shop floor workers.

---
## Dependency:
* [ROS Kinetic](http://wiki.ros.org/ROS/Installation) on Ubuntu 16.04
* [ARIAC - 2017](http://gazebosim.org/)

## Standard install via command-line
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ cd src/
$ git clone --recursive https://github.com/raviBhadeshiya/project_ariac.git
$ cd ..
$ catkin_make
```
To run qulifiers type:
```
$ rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/<FILE_NAME>.yaml ~/catkin_ws/src/project_ariac/config/final_config.yaml
```
and in new terminal type:
```
$ roslaunch project_ariac project_ariac.launch
```
---
Dropped Part Scenario:  
The robot recovery after dropping a part and complete the given Order by picking up the dropped part.
<p align="center"><img src="result/drop_part.gif"></p>

---
Conveyor picking:
<p align="center"><img src="result/conveyor.gif"></p>
