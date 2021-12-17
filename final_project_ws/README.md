This ROS workspace will allow to run an onboard pipeline in a VOXL m500 quadrotor to search and land over a moving Aruco tag.

It requieres to have vox_mpa_to_ros package installed and running on-board the VOXL Flight Companion in the YOCTO OS, as well as a docker container with ROS melodic version.

The provided workspace must be built and sourced in the docker container. The following command launches the necessary nodes to be run in offboard mode:

```
roslaunch track_and_land initialize.launch
```
