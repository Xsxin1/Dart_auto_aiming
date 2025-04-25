# rm_auto_aim

## Overview

RoboMaster 装甲板自瞄算法模块

<img src="docs/rm_vision.svg" alt="rm_vision" width="200" height="200">

该项目为 [rm_vision](https://github.com/chenjunnn/rm_vision) 的子模块

若有帮助请Star这个项目，感谢~

### License

The source code is released under a [MIT license](rm_auto_aim/LICENSE).

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

Author: Chen Jun

运行环境：Ubuntu 22.04 / ROS2 Humble (未在其他环境下测试)

![Build Status](https://github.com/chenjunnn/rm_auto_aim/actions/workflows/ros_ci.yml/badge.svg)

## Packages

- [detector](detector)

	订阅相机参数及图像流进行引导灯的识别并解算三维位置，输出识别到的引导灯在输入frame下的三维位置 (一般是以相机光心为原点的相机坐标系)

- [tracker](tracker)

	订阅识别节点发布的引导灯三维位置及飞镖的坐标转换信息，将引导灯三维位置变换到指定惯性系下，然后将引导灯目标送入跟踪器中，输出跟踪引导灯在指定惯性系下的状态

- auto_aim_interfaces

	定义了识别节点和处理节点的接口以及定义了用于 Debug 的信息

