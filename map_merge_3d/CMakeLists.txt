cmake_minimum_required(VERSION 3.1)
project(map_merge_3d)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  tf2_eigen
  tf2_ros
)

find_package(PCL REQUIRED COMPONENTS core features filters io registration visualization)

# workaround issues on debian
# see: https://github.com/ros-perception/perception_pcl/issues/139
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM PCL_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt")
endif()

# There is a bug in the Ubuntu Artful (17.10) version of the VTK package,
# where it includes /usr/include/*-linux-gnu/freetype2 in the include
# directories (which doesn't exist).  This filters down to the PCL_INCLUDE_DIRS,
# and causes downstream projects trying to use these libraries to fail to
# configure properly.  Here we remove those bogus entries so that downstream
# consumers of this package succeed.
if(NOT "${PCL_INCLUDE_DIRS}" STREQUAL "")
  foreach(item ${PCL_INCLUDE_DIRS})
    string(REGEX MATCH "/usr/include/.*-linux-gnu/freetype2" item ${item})
    if(item)
      list(REMOVE_ITEM PCL_INCLUDE_DIRS ${item})
    endif()
  endforeach()
endif()

# C++14 is supported since ROS Melodic
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
  ${PCL_INCLUDE_DIRS}
)

add_library(map_merging STATIC
  src/features.cpp
  src/graph.cpp
  src/map_merging.cpp
  src/matching.cpp
)
add_dependencies(map_merging ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_merging ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# visualising tool
add_executable(registration_visualisation
  src/registration_visualisation.cpp
  src/visualise.cpp
)
add_dependencies(registration_visualisation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(registration_visualisation map_merging ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# commandline tool for merging
add_executable(map_merge_tool
  src/map_merge_tool.cpp
)
add_dependencies(map_merge_tool ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_merge_tool map_merging ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# ROS node for online merging
add_executable(map_merge_node
  src/map_merge_node.cpp
)
add_dependencies(map_merge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_merge_node map_merging ${catkin_LIBRARIES} ${PCL_LIBRARIES})

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

  catkin_add_gtest(test_map_merging test/test_map_merging.cpp)
  target_link_libraries(test_map_merging map_merging ${catkin_LIBRARIES} ${PCL_LIBRARIES})

  # test all launch files
  roslaunch_add_file_check(launch)
endif()
