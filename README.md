## Description

This workspace provides the necessary ROS packages to operate the outdoor skid-steering robot from [Haibot Lab](https://sites.google.com/view/haibot-lab?fbclid=IwY2xjawEkHVpleHRuA2FlbQIxMAABHe2dXi4Qrxt8FIQQPHEhcPUdHKU1V9dTJYjUC7L9EI_iRUKD7Dky8Ida0Q_aem_jmdgauVqJfeysomfGbbbKQ).

## Table of Contents

* [Robotic Accessories](robotic-accessories)
* [Requirements](requirements)
  * [Environment](environment)
  * [Installing Packages](installing-packages)
* [Repository Management](repository-management)
* [Run Instructions](run-instructions)
* [Sources](sources)

## Robotic Accessories

All the hardware equipments provided for the robot are listed in the table below.

| Equipment          | Link to product                                                                                                  |
| -------------------| ---------------------------------------------------------------------------------------------------------------- |
| Robotic Controller | [ROScube-X (RQX-580/58G)](https://www.adlinktech.com/Products/ROS2_Solution/ROS2_Controller/RQX-580_58G?lang=en) |
| LiDAR 3D           | [Velodyne's Puck lidar sensor (VLP-16)](https://ouster.com/products/hardware/vlp-16)                             |
| GNSS Device        | [Duro Inertial Starter Kit](https://store.clearpathrobotics.com/products/duro-inertial-starter-kit)              |
| IMU                | [MicroStrain 3DM-GX5-AHRS](https://www.microstrain.com/inertial-sensors/3dm-gx5-25)                              |
| Vision Sensor      | [Visionary-T](https://www.sick.com/ag/en/catalog/archive/visionary-t/c/g358152)                                  |

## Requirements

### Environment

* Supported Operating System:
  * Ubuntu 18.04 LTS (Bionic Beaver)[^1](https://releases.ubuntu.com/18.04/)
  * Ubuntu 20.04 LTS (Focal Fossa)[^2](https://releases.ubuntu.com/focal/)
* Software Libraries / Toolbox:
  * ROS1 Noetic[^3](https://wiki.ros.org/ROS/Installation)
  * Gazebo Classic[^4](https://classic.gazebosim.org/)
  * Programming Languages: Python, C++

### Installing Packages

* robot_localization package
  This package is a collection of state estimation nodes, each of which is an implementation of a nonlinear state estimator for robots moving in 3D space. It contains two state estimation nodes, ekf_localization_node and ukf_localization_node. In addition, robot_localization provides navsat_transform_node, which aids in the integration of GPS data[^5](https://docs.ros.org/en/indigo/api/robot_localization/html/index.html#)

  To install the package:
  
```bash
sudo apt update
sudo apt install ros-noetic-robot-localization
```
  
* husky_simulator
  This package provides the simulation programs of Clearpath's Husky robot[^6](http://wiki.ros.org/ClearpathRobotics).

  To install the package:
  
```bash
sudo apt update
sudo apt install ros-noetic-husky-simulator
```

## Repository Management

```bash
src/
   ├── combine_data/
   ├── connect_node/
   ├── inverse_kinematic/
   ├── navigation/
   ├── pointcloud_to_laserscan/
   ├── ps3joy/
   ├── sensors_initialization/
   │   ├── duro_gps_driver/
   │   ├── microstrain_inertial/
   │   ├── sick_scan-master/
   │   ├── sick_visionary_ros/
   │   └── velodyne/
   ├── simulation/
   │   ├── cpr_gazebo/
   │   └── my_husky_sim/
   └── ...
```
