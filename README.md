# hikrobot_mv_ros

This is a ros driver package for the hikrobot mv cameras

## Overview

This driver is based on the library provided by hikrobot.
In this example, a raw image is captured and published based on the **cv_bridge** and **image_transport** packages

## Prerequisite

- Install the ros melodic package (or more recent neotic...) on your platform 
- Install the MVS 2.1.0 for Linux (https://en.hikrobotics.com/machinevision/service/download?module=0)

## Installation

### Installation from package

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

```
$ cd catkin_workspace/src
$ git clone https://github.com/rdouguet/hikrobot_mv_ros
```
		
### Building from source

```
$ cd catkin_workspace
$ catkin_make --only-pkg-with-deps hikrobot_mv_ros
$ catkin_make install --only-pkg-with-deps hikrobot_mv_ros
```

The path of hikrobot mv librairies depend of our platform architecture.
You must specified the correct path in the **CMakeLists.txt** file 

```
# the last folder depending of the architecture (64, aarch64, ...)
$ link_directories(/opt/MVS/lib/aarch64) 
$ SET(CMAKE_INSTALL_RPATH /opt/MVS/lib/aarch64)
```

### Running

A roslaunch file allows to define some parameters (frame_id, path of config file...) and start this node.

```
# Start the hikrobot camera
$ roslaunch hikrobot_mv_ros hikrobot_mv.launch
```

## Config file

* **camera.yaml** contains the camera parameter which can set during the driver initialization
