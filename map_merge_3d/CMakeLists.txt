cmake_minimum_required(VERSION 2.8.3)
project(map_merge_3d)

# C++11 is supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
)

################################################
## Declare ROS messages, services and actions ##
################################################
# we don't have any

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(registration_visualisation
  src/registration_visualisation.cpp
  src/visualise.cpp
  src/matching.cpp
  src/features.cpp
)
add_dependencies(registration_visualisation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(registration_visualisation
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# install libraries and executables
install(TARGETS registration_visualisation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

# TODO
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_map_merge_3d.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
