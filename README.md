# map_merge_3d

[![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__map_merge__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__map_merge__ubuntu_bionic_amd64)

ROS package for merging 3D point cloud maps.

Installing
----------

The package is released for ROS Melodic.

```
	sudo apt install ros-${ROS_DISTRO}-map-merge-3d
```

Building
--------

The package should build as a standard catkin package. Use rosdep to resolve
dependencies in ROS. The package is intended for ROS Melodic and newer, it
should build on all [supported platforms](http://www.ros.org/reps/rep-0003.html#melodic-morenia-may-2018-may-2023)
of ROS Melodic. Most notably, the package depends on PCL >= 1.8.

You should use the brach specific for your release i.e. `melodic-devel` for
ROS Melodic. Master branch is for the latest ROS.

WIKI
----

The package is documented at ROS Wiki.
* [map_merge_3d](http://wiki.ros.org/map_merge_3d)

COPYRIGHT
---------

The package is licensed under BSD license. See respective files for details.
