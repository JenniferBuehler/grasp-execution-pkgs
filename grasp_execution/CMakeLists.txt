cmake_minimum_required(VERSION 2.8.3)
project(grasp_execution)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  grasp_execution_msgs
  roscpp
  actionlib actionlib_msgs
  std_msgs
  trajectory_msgs
  architecture_binding
  arm_components_name_manager
  convenience_math_functions
  convenience_ros_functions
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simple_grasp_control_server simple_grasp_action_server 
  CATKIN_DEPENDS control_msgs grasp_execution_msgs roscpp
        std_msgs trajectory_msgs architecture_binding arm_components_name_manager
        convenience_math_functions convenience_ros_functions
        actionlib actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(simple_grasp_control_server
  src/SimpleGraspControlServer.cpp
)
add_dependencies(simple_grasp_control_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(simple_grasp_action_server
  src/GraspEligibilityChecker.cpp
  src/SimpleGraspActionServer.cpp
)
add_dependencies(simple_grasp_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(simple_grasp_control_server_node src/simple_grasp_control_server_node.cpp)
add_dependencies(simple_grasp_control_server_node simple_grasp_control_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(simple_grasp_action_server_node src/simple_grasp_action_server_node.cpp)
add_dependencies(simple_grasp_action_server_node simple_grasp_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(simple_grasp_control_client test/simple_grasp_control_client.cpp)
add_dependencies(simple_grasp_control_client simple_grasp_control_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(simple_grasp_control_server
  ${catkin_LIBRARIES}
)
target_link_libraries(simple_grasp_action_server
  ${catkin_LIBRARIES}
)

target_link_libraries(simple_grasp_control_server_node
  simple_grasp_control_server
  ${catkin_LIBRARIES}
)

target_link_libraries(simple_grasp_action_server_node
  simple_grasp_action_server
  ${catkin_LIBRARIES}
)

target_link_libraries(simple_grasp_control_client
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS 
    simple_grasp_control_server simple_grasp_control_server_node simple_grasp_control_client
    simple_grasp_action_server simple_grasp_action_server_node  
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_grasp_execution.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
