# detector

- [LightDetectorNode](#basedetectornode)
  - [Detector](#detector)
  - [PnPSolver](#pnpsolver)

## 识别节点

订阅相机参数及图像流进行引导灯的识别并解算三维位置，输出识别到的引导灯在输入frame下的三维位置 (一般是以相机光心为原点的相机坐标系)

### LightDetectorNode
装甲板识别节点

包含[Detector](#detector)
包含[PnPSolver](#pnpsolver)

订阅：
- 相机参数 `/camera_info`
- 彩色图像 `/image_raw`

发布：
- 识别目标 `/detector/lights`

动态参数：
- 是否开启 opencv 调试 `cv_detector_debug_`
- 是否开启 foxglave 可视化 `fg_detector_debug_`
- 二值化的最小阈值 `binary_thres`
- 多边形拟合精度 `epsilon`
- 矩形长宽高初筛选阈值 `contours_ratio_min`，`contours_ratio_max`
- 矩形面积筛选阈值 `contours_area_min`，`contours_area_max`
- 椭圆面积后筛选阈值 `ellipse_area_min`，`ellipse_area_max`
- 椭圆长宽高后筛选阈值 `ellipse_ratio_min`，`ellipse_ratio_max`
- 椭圆填充率筛选阈值 `filled_ratio_min`，`filled_ratio_max`
- 识别目标颜色 `detect_color`


## Detector
引导灯识别器

### preprocessImage
预处理

| ![](docs/raw.png) | ![](docs/diff.png) | 
| :---------------: | :-------------------: 
|       原图        |    G与R通道作差     |   

由于一般工业相机的动态范围不够大，得到的相机图像中引导灯中心就会过曝，灯条中心的像素点的值往往都是 R=B，考虑到引导灯的形状易受环境影响，选择 G 与 R 通道作差，去除白色成分，得到一张绿色占主要成分的灰度图，再进行灰度二值化，得到一张二值化图像，用于后续的灯条轮廓提取，其中引导灯应为圆环状。

### findLights
寻找灯条

通过 findContours 得到轮廓及其层级，筛选出圆环内层轮廓，再通过 approxPolyDP  拟合，通过外接矩形，对其进行长宽比和面积的初筛选，可以初步筛除形状不满足的亮斑，减少计算量。再通过椭圆拟合，对椭圆的长宽比、面积、填充率进行筛选，得到最终灯条轮廓。

后续会通过引导灯上方装甲板进行筛选，得到最终结果。

## PnPSolver
PnP解算器

[Perspective-n-Point (PnP) pose computation](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)

PnP解算器将 `cv::solvePnP()` 封装，接口中传入 `Light` 类型的数据即可得到 `geometry_msgs::msg::Point` 类型的三维坐标。

考虑到引导灯为椭圆，椭圆上下左右顶点在一个平面上，在PnP解算方法上我们选择了 `cv::SOLVEPNP_IPPE` (Method is based on the paper of T. Collins and A. Bartoli. ["Infinitesimal Plane-Based Pose Estimation"](https://link.springer.com/article/10.1007/s11263-014-0725-5). This method requires coplanar object points.)

由于引导灯是对称的，因此解出来的旋转矩阵是有多解的，需要后续增加一个参考点筛选。
且由于引导灯的光斑无法预估，这里的解算距离结果误差较大，后续添加装甲板作为参考也许效果较好。目前仅使用坐标点计算相对于飞镖飞行面的yaw。