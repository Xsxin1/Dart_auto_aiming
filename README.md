# rm_auto_aim

## Overview

RoboMaster 引导灯自瞄系统

<img src="docs/rm_vision.svg" alt="rm_vision" width="200" height="200">

该项目基于 [rm_vision](https://github.com/chenjunnn/rm_vision) 框架，使用北极熊的相机驱动[hik_camera_ros2_driver](https://github.com/SMBU-PolarBear-Robotics-Team/hik_camera_ros2_driver),是用于RoboMaster飞镖镖架的引导灯识别，仅在中期考核中测试过，水平有限，仅供参考.

若有帮助请Star这个项目，感谢~

### License

The source code is released under a [MIT license](rm_auto_aim/LICENSE).

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

Author: Xsxin 

Email: xsxin123@163.com

运行环境：Ubuntu 22.04 / ROS2 Humble (未在其他环境下测试)

![Build Status](https://github.com/chenjunnn/rm_auto_aim/actions/workflows/ros_ci.yml/badge.svg)


## Packages

- rm_auto_aim

	用于识别和跟踪引导灯，发布目标位置

- rm_gimbal_description

	描述飞镖的结构，发布静态tf变换

- rm_serial_driver

	用于与下位机通信

- rm_vision

	包含启动识别节点和处理节点的默认参数文件及 launch 文件
