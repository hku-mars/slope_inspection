# IPC

**Integrated Planning and Control for Quadrotor Navigation in Presence of suddenly appearing Objects and Disturbances**

## Overview

Author: [Wenyi Liu](https://github.com/FENYUN323), [Yunfan Ren](https://github.com/RENyunfan) and [Fu Zhang](https://www.mech.hku.hk/academic-staff/Zhang-F) from [HKU MARS Lab](https://mars.hku.hk/).

Paper: To be released soon.

Code: [Github](https://github.com/hku-mars/IPC)

Video Links: [youtube](https://www.youtube.com/watch?v=EZFxTkqqat4), [Bilibili](https://www.bilibili.com/video/BV1NM4y117TH)

Click for the video demo.

[![Video Demo](./img/out.png)](https://www.youtube.com/watch?v=EZFxTkqqat4)

## 1 About IPC

The **IPC** is an integrated planning and control framework for quadrotors.

The **IPC's characteristic**:

* Safety (Using safe flight corridor as hard constraints)
* Low latency (Can run at 100Hz)
* Strong disturbance rejection capability

Using **IPC**, the quadrotor can:

* Avoid sudden crossing object
* Fly safely under disturbances (i.e., external forces and wind disturbances)
* Navigate autonomously in dense woods

## 2 Prerequisited

### 2.1 Ubuntu and ROS

Ubuntu 18.04~20.04, [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 2.2 PCL and Eigen

PCL >= 1.6, follow [PCL Installation](https://pointclouds.org)

Eigen >= 3.3.4, follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page)

### 2.3 OSQP and OSQP-Eigen

Select the version below v0.6.3

* [osqp-github](https://github.com/osqp/osqp)
* [osqp-document](https://osqp.org/docs/get_started/sources.html)
* [osqp-releases(below v0.6.3)](https://github.com/osqp/osqp/releases/tag/v0.6.3)
* [qdldl-releases(below v0.1.7)](https://github.com/osqp/qdldl/releases/tag/v0.1.7)

Install OSQP
```
git clone --recursive https://github.com/osqp/osqp
cd osqp
mkdir build
cd build
cmake ..
sudo make install
```

* [osqp-eigen-github](https://github.com/robotology/osqp-eigen)

Install OSQP-Eigen
```
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build
cd build
cmake ..
sudo make
sudo make install
```

### 2.4 Other

A debug tool: *backward.cpp*

Installation
```
sudo apt-get install libdw-dev
wget https://raw.githubusercontent.com/bombela/backward-cpp/master/backward.hpp
sudo mv backward.hpp /usr/include
```

## 3 Make

```
mkdir -p IPC_ws/src
cd IPC_ws/src
git clone https://github.com/hku-mars/IPC.git
cd ..
catkin_make
```

## 4 Run and test
```
source devel/setup.bash
roslaunch ipc planning.launch
```