cmake_minimum_required(VERSION 2.8.3)
project(project_ariac)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  osrf_gear
  roscpp
  sensor_msgs
  std_srvs
  trajectory_msgs
  std_msgs
  message_generation
  control_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()
# generate_messages(
#     DEPENDENCIES
#     std_msgs sensor_msgs trajectory_msgs #VacuumGripperControl
# )

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS osrf_gear control_msgs)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(project_ariac_node_1b src/qual1b.cpp src/manageOrder.cpp src/moveArm.cpp)
add_dependencies(project_ariac_node_1b ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_generate_messages_cpp})

target_link_libraries(project_ariac_node_1b ${catkin_LIBRARIES})

add_executable(project_ariac_node_1a src/qual1a.cpp src/manageOrder.cpp src/moveArm.cpp)
add_dependencies(project_ariac_node_1a ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_generate_messages_cpp})

target_link_libraries(project_ariac_node_1a ${catkin_LIBRARIES})

