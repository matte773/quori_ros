# quori_ros

This catkin workspace contains all of the ROS packages necessary for full operation of the Quori robot platform from UPenn.

## This quori_ros
This fork of quori_osu was created and maintained by Matthew Miller in 2023 when beginning work on the OSU Quori (quori6). This has since added the 2024 REU as well as the Quori Q&A project code to it. The majority of additions are added to a quori_osu packing located in quori_ros/src.

## Prerequisites

The package is compatible with **Ubuntu 16.04 / ROS Kinetic** and **Ubuntu 20.04 / ROS Noetic**, built with the **Catkin** system. Make sure you have the right environment configured.

### External Dependencies

Some ROS packages are required for use of this workspace:
```
sudo apt-get install ros-${ROS_DISTRO}-sound-play ros-${ROS_DISTRO}-rgbd-launch ros-${ROS_DISTRO}-libuvc ros-${ROS_DISTRO}-libuvc-camera ros-${ROS_DISTRO}-libuvc-ros
```

## Initial Setup

```
git clone https://github.com/Quori-ROS/quori_ros.git
cd quori_ros
git submodule init
catkin_make
. ./devel/setup.sh
```
