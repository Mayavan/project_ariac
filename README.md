# project_ariac
[![Packagist](https://img.shields.io/pypi/l/Django.svg)](LICENSE)
---
* If you are someone who doesn’t know how to use GitHub, then once go though this tutorials.[click here](https://try.github.io)
* Upload your code on your branch and **DON'T MERGE ANY BRANCH TO MASTER WITHOUT PEER-REVIEW!**
* Please try to follow [Google Cpp style Guide](https://google.github.io/styleguide/cppguide.html) so every one can understand the code!
* I belive we are gonna devlope whole final project on top of this project so try to use OOP as well as [CPP 14](https://github.com/AnthonyCalandra/modern-cpp-features).

## Standard install via command-line
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ cd src/
$ git clone --recursive https://github.com/raviBhadeshiya/enigma.git
$ git checkout <name_of_branch>
$ cd ..
$ catkin_make
```


To run qulifier1 type:
```
$ rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual1a.yaml ~/catkin_ws/src/project_ariac/config/qual1a_config.yaml 
```
and in new terminal type:
```
$ roslaunch project_ariac project_ariac_qual_1a.launch
```
