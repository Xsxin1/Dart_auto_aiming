# rm_gimbal_description
RoboMaster 视觉自瞄系统所需的 URDF

<img src="docs/rm_vision.svg" alt="rm_vision" width="200" height="200">

该项目为 [rm_vision](https://github.com/chenjunnn/rm_vision) 的子模块

## 坐标系定义

单位和方向请参考 https://www.ros.org/reps/rep-0103.html

chassis_link: 飞镖底盘

muzzle_link: 飞镖发射点，x轴为水平向前

camera_link: 相机坐标系，x轴向前

camera_joint: 表述相机到底盘的变换关系

camera_optical_joint: 表述以 z 轴为前方的相机坐标系转换为 x 轴为前方的相机坐标系的旋转关系

## 使用方法

修改 [urdf/rm_gimbal.urdf.xacro](urdf/rm_gimbal.urdf.xacro) 中的 `gimbal_camera_transfrom` 

