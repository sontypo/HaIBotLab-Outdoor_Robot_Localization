## Description

This workspace provides the necessary ROS packages to operate the outdoor skid-steering robot from [Haibot Lab](https://sites.google.com/view/haibot-lab?fbclid=IwY2xjawEkHVpleHRuA2FlbQIxMAABHe2dXi4Qrxt8FIQQPHEhcPUdHKU1V9dTJYjUC7L9EI_iRUKD7Dky8Ida0Q_aem_jmdgauVqJfeysomfGbbbKQ).

## Table of Contents

* [Robotic Accessories](#robotic-accessories)
* [Requirements](#requirements)
  * [Environment](#environment)
  * [Installing Packages](#installing-packages)
* [Repository Management](#repository-management)
* [Run Instructions](#run-instructions)
* [References](#references)

## Robotic Accessories

All the hardware equipments provided for the robot are listed in the table below.

| Equipment          | Link to product                                                                                                              |
| -------------------| ---------------------------------------------------------------------------------------------------------------------------- |
| Robotic Controller | [ROScube-X (RQX-580/58G)](https://www.adlinktech.com/Products/ROS2_Solution/ROS2_Controller/RQX-580_58G?lang=en)             |
| LiDAR 3D           | [Puck lidar sensor (VLP-16)](https://ouster.com/products/hardware/vlp-16)                                                    |
| LiDAR 2D           | [LiDAR sensors (NAV245)](https://www.sick.com/cl/en/catalog/products/lidar-and-radar-sensors/lidar-sensors/nav2xx/c/g356151) |
| GNSS Device        | [Duro Inertial Starter Kit](https://store.clearpathrobotics.com/products/duro-inertial-starter-kit)                          |
| IMU                | [MicroStrain 3DM-GX5-AHRS](https://www.microstrain.com/inertial-sensors/3dm-gx5-25)                                          |
| Vision Sensor      | [SICK Visionary-T](https://www.sick.com/ag/en/catalog/archive/visionary-t/c/g358152)                                              |

## Requirements

### Environment

* Supported Operating System:
  * Ubuntu 18.04 LTS (Bionic Beaver)[^1]
  * Ubuntu 20.04 LTS (Focal Fossa)[^2]
* Software Libraries / Toolbox:
  * ROS1 Noetic[^3]
  * Gazebo Classic[^4]
  * Programming Languages: Python, C++

### Installing Packages

* robot_localization package
  This package is a collection of state estimation nodes, each of which is an implementation of a nonlinear state estimator for robots moving in 3D space. It contains two state estimation nodes, ekf_localization_node and ukf_localization_node. In addition, robot_localization provides navsat_transform_node, which aids in the integration of GPS data[^5].

  To install the package:
  
  ```bash
  sudo apt update
  sudo apt install ros-noetic-robot-localization
  ```
  
* husky_simulator
  This package provides the simulation programs of Clearpath's Husky robot[^6].

  To install the package:
  
  ```bash
  sudo apt update
  sudo apt install ros-noetic-husky-simulator
  ```

## Repository Management

Below is a tree diagram showing the partition of the packages in this repository.

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

The purpose of each package is described in the table below:

| Package                                                                        | Description                                                                                                      |
| -------------------------------------------------------------------------------| ---------------------------------------------------------------------------------------------------------------- |
| [combine_data](./src/combine_data)                                             | This package initializes the EKF filter and NavSat satellite's data transformation for robot localization in outdoor environments. |
| [connect_node](./src/connect_node)                                             | This package sets up the connection method with the microcontroller on the robot's mobile platform, enabling communication and control data transmission to the actuators. |
| [inverse_kinematic](./src/inverse_kinematic)                                   | This package provides the inverse kinematics model for a skid-steering robot, supporting motion control integration and odometry data calculation. |
| [pointcloud_to_laserscan](./src/pointcloud_to_laserscan)                       | This package is responsible for converting point cloud data from 3D-capable sensors into laser scan data suitable for specific tasks. |
| [ps3joy](./src/ps3joy)                                                         | This package is responsible for establishing a connection with a PS3 joystick device, allowing users to control the robot in manual mode. |
| [duro_gps_driver](./src/sensors_initialization/duro_gps_driver)                | This package provides a ROS driver for initializing and acquiring data from the Duro Inertial Starter Kit. It returns the data in a format compatible with ROS, allowing users to utilize and program with it. |
| [microstrain_inertial](./src/sensors_initialization/microstrain_inertial)      | This package provides a ROS driver for initializing and acquiring data from the MicroStrain 3DM-GX5-AHRS. It returns the data in a format compatible with ROS, allowing users to utilize and program with it. |
| [sick_scan-master](./src/sensors_initialization/sick_scan-master)              | This package provides a ROS driver for initializing and acquiring data from the LiDAR sensors (NAV245). It returns the data in a format compatible with ROS, allowing users to utilize and program with it. |
| [sick_visionary_ros](./src/sensors_initialization/sick_visionary_ros)          | This package provides a ROS driver for initializing and acquiring data from the SICK Visionary-T. It returns the data in a format compatible with ROS, allowing users to utilize and program with it. |
| [velodyne](./src/sensors_initialization/velodyne)                              | This package provides a ROS driver for initializing and acquiring data from the Puck lidar sensor (VLP-16). It returns the data in a format compatible with ROS, allowing users to utilize and program with it. |
| [cpr_gazebo](./src/sensors_initialization/cpr_gazebo)                          | [Visionary-T](https://www.sick.com/ag/en/catalog/archive/visionary-t/c/g358152)                                  |
| [my_husky_sim](./src/sensors_initialization/my_husky_sim)                      | [Visionary-T](https://www.sick.com/ag/en/catalog/archive/visionary-t/c/g358152)                                  |

## Run Instructions 

## References
[^1]: https://releases.ubuntu.com/18.04/
[^2]: https://releases.ubuntu.com/focal/
[^3]: https://wiki.ros.org/ROS/Installation
[^4]: https://classic.gazebosim.org/
[^5]: https://docs.ros.org/en/indigo/api/robot_localization/html/index.html#
[^6]: http://wiki.ros.org/ClearpathRobotics
