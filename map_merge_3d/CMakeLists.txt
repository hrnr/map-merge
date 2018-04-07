cmake_minimum_required(VERSION 2.8.3)
project(map_merge_3d)

# C++14 should be de facto supported in ROS Lunar and newer
# we need C++14 for templated lambdas
add_compile_options(-std=c++14)

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

add_library(map_merging STATIC
  src/matching.cpp
  src/features.cpp
)
add_dependencies(map_merging ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_merging ${catkin_LIBRARIES})

# visualising tool
add_executable(registration_visualisation
  src/registration_visualisation.cpp
  src/visualise.cpp
)
add_dependencies(registration_visualisation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(registration_visualisation
  map_merging
  ${catkin_LIBRARIES}
)

# commandline tool for merging
add_executable(map_merge_tool
  src/map_merge_tool.cpp
)
add_dependencies(map_merge_tool ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_merge_tool
  map_merging
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
