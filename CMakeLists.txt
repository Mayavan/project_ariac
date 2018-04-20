cmake_minimum_required(VERSION 2.8.3)
project(project_ariac)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  osrf_gear
  roscpp
  sensor_msgs
  std_srvs
  std_msgs
  control_msgs
  geometry_msgs
  tf2
  moveit_core
  moveit_ros_planning_interface
  # message_generation
  actionlib
)

# add_service_files(
#   FILES
#   pickup.srv
#   place.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs geometry_msgs#   move_base_msgs#   sensor_msgs#
# )


catkin_package()

    # INCLUDE_DIRS include
    # CATKIN_DEPENDS osrf_gear control_msgs geometry_msgs
    # )


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(project_ariac_node src/project_ariac_node.cpp src/UR10_Control.cpp src/Manager.cpp src/Interface.cpp)

add_dependencies(project_ariac_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_generate_messages_cpp})

target_link_libraries(project_ariac_node ${catkin_LIBRARIES})

add_executable(node src/Node.cpp src/Interface.cpp src/Conveyor.cpp)

add_dependencies(node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_generate_messages_cpp})

target_link_libraries(node ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS script/ariac_example_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS project_ariac_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
 config/sample_gear_conf.yaml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ariac_example.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
