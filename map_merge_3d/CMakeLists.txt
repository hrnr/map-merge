cmake_minimum_required(VERSION 2.8.3)
project(map_merge_3d)

# C++14 should be de facto supported in ROS Lunar and newer
# we need C++14 for templated lambdas
add_compile_options(-std=c++14)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  tf2_eigen
  tf2_ros
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
  src/features.cpp
  src/graph.cpp
  src/map_merging.cpp
  src/matching.cpp
)
add_dependencies(map_merging ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_merging ${catkin_LIBRARIES})

# visualising tool
add_executable(registration_visualisation
  src/registration_visualisation.cpp
  src/visualise.cpp
)
add_dependencies(registration_visualisation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(registration_visualisation map_merging ${catkin_LIBRARIES})

# commandline tool for merging
add_executable(map_merge_tool
  src/map_merge_tool.cpp
)
add_dependencies(map_merge_tool ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_merge_tool map_merging ${catkin_LIBRARIES})

# ROS node for online merging
add_executable(map_merge_node
  src/map_merge_node.cpp
)
add_dependencies(map_merge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_merge_node map_merging ${catkin_LIBRARIES})

#############
## Install ##
#############

# install libraries and executables
install(
  TARGETS map_merge_node map_merge_tool map_merging registration_visualisation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install roslaunch files
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  # TODO download test data

  catkin_add_gtest(test_map_merging test/test_map_merging.cpp)
  # TODO ensure that test data are downloaded before we run tests
  target_link_libraries(test_map_merging map_merging ${catkin_LIBRARIES})

  # test all launch files
  roslaunch_add_file_check(launch)
endif()
