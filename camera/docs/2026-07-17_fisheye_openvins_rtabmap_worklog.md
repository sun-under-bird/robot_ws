# 2026-07-17 鱼眼双目、OpenVINS 与 RTAB-Map 工作记录和排障手册

本文记录本次鱼眼双目重新标定、相机与 IMU 联合标定、ROS 2 `CameraInfo` 适配、OpenVINS/RTAB-Map 集成、点云降噪、源码编译，以及漂移和回环排查的结果。它既是当天的工作总结，也可作为以后复现实验和定位问题的操作手册。

## 1. 当前结论

当前系统链路已经跑通：

```text
TST 双目鱼眼相机（20 Hz） ─┐
                            ├─ OpenVINS 双目惯性里程计 ─ odom → imu_link
WIT IMU（200 Hz）───────────┘
                                     │
鱼眼整流图像 ────────────────────────┴─ RTAB-Map ─ map → odom
```

目前使用的主要方案如下：

- 两个相机都使用 Kalibr 的 `pinhole-equi` 模型重新标定。
- 原始图像发布 `equidistant` 类型的 `CameraInfo`，再由 `image_proc/rectify_node` 生成校正图像。
- 里程计使用 RTAB-Map 内的 OpenVINS 后端，即 `Odom/Strategy=10`。
- RTAB-Map 使用校正后的双目图像做回环检测和建图。
- 稠密双目改用较保守的 StereoBM，并限制点云距离、体素分辨率和离群点过滤，飞点明显减少。
- 最新一次运行没有出现前一次那种持续漂移，回环也能产生实际的 `map -> odom` 修正。
- 当前最明显的剩余风险是曝光时间过长。默认曝光值 `580` 对应约 `58 ms`，已经超过 20 Hz 相机的 `50 ms` 帧周期，快速甩动时容易产生径向运动模糊并导致视觉特征骤降。

## 2. 主要文件位置

### 2.1 启动和参数文件

| 用途 | 路径 |
| --- | --- |
| 一键启动相机、IMU、整流、TF 和建图 | `/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/launch/equi_imu_rtabmap.launch.py` |
| OpenVINS + RTAB-Map 节点启动 | `/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/launch/rtabmap_openvins_stereo_mapping.launch.py` |
| OpenVINS 和 RTAB-Map 参数 | `/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/config/rtabmap_openvins_mapping_params.yaml` |
| 可动态传参的相机启动文件 | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/launch/tst_openvins_20fps.launch.py` |
| 左目鱼眼 `CameraInfo` | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/config/left_equi.yaml` |
| 右目鱼眼 `CameraInfo` | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/config/right_equi.yaml` |

### 2.2 标定文件

| 用途 | 路径 |
| --- | --- |
| 联合标定报告 | `/home/bird/kalibr_data/cam_imu_repeat_01-report-imucam.pdf` |
| 联合标定文本结果 | `/home/bird/kalibr_data/cam_imu_repeat_01-results-imucam.txt` |
| 联合标定相机链 | `/home/bird/kalibr_data/cam_imu_repeat_01-camchain-imucam.yaml` |
| 双目相机标定相机链 | `/home/bird/kalibr_data/cam_imu_repeat_01-camchain.yaml` |
| Kalibr 转 ROS `CameraInfo` 脚本 | `/home/bird/kalibr_data/kalibr_camchain_to_ros_camera_info.py` |

当前运行配置来自联合标定结果 `cam_imu_repeat_01-camchain-imucam.yaml`。不要把另一轮双目标定的内参、联合标定的外参和旧的 `CameraInfo` 混用。

### 2.3 运行产物

| 用途 | 路径 |
| --- | --- |
| 当前 RTAB-Map 数据库 | `/home/bird/.ros/rtabmap_openvins_equi_mapping.db` |
| 原始里程计轨迹 | `/home/bird/.ros/rtabmap_openvins_equi_mapping_odom.txt` |
| 图优化后轨迹 | `/home/bird/.ros/rtabmap_openvins_equi_mapping_slam.txt` |
| ROS 2 日志根目录 | `/home/bird/.ros/log` |

## 3. 今日完成的工作

### 3.1 两个 bag 都改用鱼眼模型重新标定

重新标定后，两目都为：

```text
camera model: pinhole
distortion model: equidistant
resolution: 640 × 480
```

联合标定的重投影误差：

| 项目 | mean | median | std |
| --- | ---: | ---: | ---: |
| cam0 重投影误差 | 0.2694 px | 0.2388 px | 0.1627 px |
| cam1 重投影误差 | 0.3001 px | 0.2880 px | 0.1507 px |
| 陀螺仪残差 | 0.02181 rad/s | 0.01476 rad/s | 0.02477 rad/s |
| 加速度计残差 | 0.07979 m/s² | 0.06028 m/s² | 0.07003 m/s² |

两目重投影误差约为 0.27～0.30 像素，标定结果可用。实际 VIO 效果还会受到时间同步、曝光、图像模糊、IMU 噪声和机械刚性的影响，因此不能只看重投影误差。

### 3.2 当前内参

左目 `cam0`：

```text
fx = 360.5344018354612
fy = 359.6366567847900
cx = 354.9278186582314
cy = 218.1833698529925
D  = [0.04861349984471425,
      0.06158474655498857,
     -0.10484697988816646,
      0.035492820261732505]
```

右目 `cam1`：

```text
fx = 360.2933689816742
fy = 358.7138589609613
cx = 308.5248443363815
cy = 256.2725050109070
D  = [0.0521131056420278,
      0.053723019069685476,
     -0.07018962018346515,
      0.009287572724957862]
```

双目基线长度：

```text
baseline = 0.04944354134483784 m ≈ 49.44 mm
```

整流后的公共投影参数：

```text
fx' = fy' = 314.03373868836536
cx' = 329.3354336108548
cy' = 236.9322986618604
右目 P[0,3] = -15.526940142512196
```

可以用下式核对整流后的基线：

```text
baseline = -P_right[0,3] / P_right[0,0]
         = 15.5269401425 / 314.0337386884
         ≈ 0.0494435 m
```

### 3.3 当前相机—IMU 外参

ROS TF 中使用 `imu_link` 作为父坐标系，`cam0`、`cam1` 作为子坐标系。对应 Kalibr 报告里的 `T_ic`，即把相机坐标中的点变换到 IMU 坐标系。

`imu_link -> cam0`：

```text
translation (m):
x = -0.042963489
y = -0.025828586
z = -0.026652652

quaternion (x, y, z, w):
qx = -0.511529228
qy = -0.489185607
qz =  0.495066366
qw =  0.503929146
```

`imu_link -> cam1`：

```text
translation (m):
x = -0.042918185
y =  0.023611724
z = -0.026089266

quaternion (x, y, z, w):
qx = -0.502205097
qy = -0.501198054
qz =  0.497948507
qw =  0.498635976
```

#### 为什么相机在 IMU 前面，x 却是负数

不能只根据外参平移向量中的 `x` 正负判断物理上的“前后”。原因是：

1. Kalibr 的 IMU 坐标轴不一定与机器人 `base_link` 的前左上坐标约定一致。
2. 上面的平移是在 **IMU 坐标系轴方向** 下表达的。
3. 两个坐标系之间还存在接近 90° 的旋转，必须把完整旋转矩阵和平移一起理解。

所以负 `x` 并不说明外参方向写反。装到机器狗后，应另外定义符合机器人约定的 `base_link -> imu_link` 固定变换，不能直接把 `imu_link` 的 x 轴当作机器人前方。

### 3.4 相机和 IMU 时间偏移

Kalibr 结果：

```text
cam0: t_imu = t_cam + 0.0307346127 s
cam1: t_imu = t_cam + 0.0307262011 s
```

双目共用硬件时间戳，因此驱动使用两者平均值：

```text
camera_time_offset_ms = 30.730406888559322
```

含义是发布相机消息时，需要按 `t_imu = t_cam + shift` 的方向对齐，而不是简单地只看数值绝对值。时间偏移方向写反，低速时可能看不明显，快速转动时会显著破坏 VIO。

## 4. 鱼眼 `CameraInfo` 的处理方法

原转换脚本只接受 `pinhole-radtan`，因此遇到：

```text
转换失败：cam0 不是 radtan 畸变模型，本脚本仅处理 pinhole-radtan
```

脚本现已支持 `pinhole-equi`：

- 原始内参继续写入 `K`。
- 原始四个鱼眼畸变参数写入 `D`。
- `distortion_model` 必须写为 `equidistant`。
- 使用 OpenCV `cv2.fisheye.stereoRectify()` 生成双目的 `R` 和 `P`。
- 右目投影矩阵 `P` 包含基线项。

生成当前 `CameraInfo` 的命令：

```bash
python3 ~/kalibr_data/kalibr_camchain_to_ros_camera_info.py \
  --camchain ~/kalibr_data/cam_imu_repeat_01-camchain-imucam.yaml \
  --left-out ~/robot_ws/imu_ws/src/stereo_v4l2_camera/config/left_equi.yaml \
  --right-out ~/robot_ws/imu_ws/src/stereo_v4l2_camera/config/right_equi.yaml
```

鱼眼模型启用后，相机内参发布必须跟着修改：原始图像不能再发布 `plumb_bob`/radtan 参数。当前发布流程是：

```text
/cam0/image_raw + /cam0/camera_info(equidistant)
                 ↓ image_proc/rectify_node
             /cam0/image_rect

/cam1/image_raw + /cam1/camera_info(equidistant)
                 ↓ image_proc/rectify_node
             /cam1/image_rect
```

只有下游确实订阅 `/cam0/image_rect` 和 `/cam1/image_rect` 时，才能设置：

```text
Rtabmap/ImagesAlreadyRectified = true
```

如果下游直接使用 `image_raw`，却又把该参数设为 `true`，双目极线约束和深度都会出错。

运行后核对：

```bash
ros2 topic echo /cam0/camera_info --once
ros2 topic echo /cam1/camera_info --once
ros2 topic hz /cam0/image_rect
ros2 topic hz /cam1/image_rect
```

重点检查：

- `distortion_model` 是 `equidistant`。
- `D` 有 4 个参数。
- 左右目 `P` 的 `fx/fy/cx/cy` 一致。
- 左目 `P[0,3]` 为 0，右目该项为负数。
- 左右校正图像频率都稳定在约 20 Hz。

## 5. 从 D435i 启动文件换成自定义相机

原 D435i 建图文件依赖 RealSense 自带的双目、IMU、坐标系和话题。换成自定义相机后，需要替换四类输入：

1. 用 `tst_openvins_20fps.launch.py` 启动 TST 双目相机。
2. 用 WIT 驱动发布 `/imu/data_raw`。
3. 用 `image_proc` 对两目 `equidistant` 图像进行整流。
4. 发布标定得到的 `imu_link -> cam0` 和 `imu_link -> cam1` 静态 TF。

当前一键启动文件已经完成这些连接：

```bash
ros2 launch stereo_camera_pkg_py equi_imu_rtabmap.launch.py
```

默认 TF 结构应为：

```text
map
└── odom
    └── imu_link
        ├── cam0
        └── cam1
```

其中：

- OpenVINS 发布 `odom -> imu_link`。
- RTAB-Map 发布 `map -> odom`。
- 标定静态 TF 发布 `imu_link -> cam0/cam1`。
- 不需要再单独发布一个“相机位姿”话题；TF 树已经表达相机相对地图的位姿。

部署到机器狗后，建议改为：

```text
map -> odom -> base_link -> imu_link -> cam0/cam1
```

此时必须由 URDF、`robot_state_publisher` 或静态 TF 提供准确的 `base_link <-> imu_link` 刚性变换，然后启动时传入：

```bash
ros2 launch stereo_camera_pkg_py equi_imu_rtabmap.launch.py \
  base_frame_id:=base_link
```

在 RViz 中要把 `Fixed Frame` 设为 `map` 才能看到回环带来的全局修正。若设为 `odom`，画面只显示连续的局部里程计，看起来会像“回环没有修正”。

## 6. 相机参数改为启动时动态可调

`tst_openvins_20fps.launch.py` 已把常用 V4L2 参数声明为 launch 参数，可以在不改源码的情况下做 A/B 测试。

查看全部参数及默认值：

```bash
ros2 launch stereo_v4l2_camera tst_openvins_20fps.launch.py --show-args
```

单独测试相机：

```bash
ros2 launch stereo_v4l2_camera tst_openvins_20fps.launch.py \
  exposure_time_absolute:=280 \
  gain:=128 \
  sharpness:=64 \
  brightness:=50 \
  auto_exposure:=1
```

完整建图启动文件会包含这个相机启动文件。若需要从最外层直接透传相机参数，可继续在 `equi_imu_rtabmap.launch.py` 中增加同名 launch argument；否则可以先单独启动传感器，再用：

```bash
ros2 launch stereo_camera_pkg_py equi_imu_rtabmap.launch.py \
  start_sensors:=false
```

当前相机默认值：

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| 分辨率 | 1280×480 | 水平拼接的两幅 640×480 图像 |
| 像素格式 | YUYV | USB 3 下优先使用 |
| 帧率 | 20 Hz | 双目同步发布 |
| `exposure_time_absolute` | 580 | V4L2 单位为 100 μs，即约 58 ms |
| `gain` | 128 | 过高会增加噪点 |
| `sharpness` | 64 | 过高会产生锐化光晕和假纹理 |
| `brightness` | 50 | 数字提亮不能改善信噪比 |
| `auto_exposure` | 1 | 手动曝光 |
| 自动白平衡 | false | 避免帧间外观变化 |
| 连续自动对焦 | 0 | VIO 必须保持固定内参 |
| `focus_absolute` | 359 | 当前固定焦点 |

### 曝光优化优先级

20 Hz 的帧周期是 50 ms，当前 58 ms 曝光在快速运动时很危险。建议先按以下顺序调试：

1. 把曝光降到 `250～300`，即 25～30 ms。
2. 优先增加环境照明。
3. 图像仍暗时，小幅提高 gain，并观察暗部噪声。
4. 保持自动曝光、自动白平衡和自动对焦关闭。
5. 每次只改变一个参数，并保存对应数据库和日志。

## 7. RTAB-Map/OpenVINS 当前参数思路

### 7.1 OpenVINS

当前关键参数：

```text
Odom/Strategy                     = 10
OdomOpenVINS/UseStereo            = true
OdomOpenVINS/UseKLT               = true
OdomOpenVINS/NumPts               = 200
OdomOpenVINS/MinPxDist            = 15
FAST/Threshold                    = 20
OdomOpenVINS/MaxClones            = 11
OdomOpenVINS/MaxSLAM              = 50
OdomOpenVINS/MaxMSCKFInUpdate     = 50
OdomOpenVINS/TryZUPT              = true
```

IMU 噪声参数与 Kalibr 使用的 WIT IMU 配置保持一致：

```text
AccelerometerNoiseDensity = 0.02
AccelerometerRandomWalk   = 0.002
GyroscopeNoiseDensity     = 0.0015
GyroscopeRandomWalk       = 0.00015
```

当前没有启用在线标定：相机内参、外参、时间偏移和 IMU 内参都固定使用离线标定结果。这样更适合已经完成良好联合标定的刚性传感器组件。

### 7.2 RTAB-Map 回环和图优化

当前关键参数：

```text
Rtabmap/DetectionRate       = 1 Hz
Rtabmap/LoopThr             = 0.11
Vis/MinInliers              = 20
Vis/FeatureType             = 8  (ORB)
Vis/MaxFeatures             = 1000
RGBD/OptimizeMaxError       = 3.0
Optimizer/Strategy          = 1  (g2o)
Optimizer/Iterations        = 20
Optimizer/GravitySigma      = 0.3
Reg/Force3DoF               = false
```

不要为了让错误回环“通过”而直接增大 `RGBD/OptimizeMaxError`。前一次失败运行中，部分候选回环虽然图像相似，但加入图以后与已有 OpenVINS 轨迹冲突，角度误差比达到 19～35，远大于阈值 3，RTAB-Map 拒绝这些约束是正确的。真正应该解决的是 VIO 漂移、运动模糊或错误匹配来源。

### 7.3 机器狗是否要启用 `Reg/Force3DoF`

四足机器人建议保持：

```text
Reg/Force3DoF = false
```

机器狗行走时会有机身俯仰和横滚，地面也可能有坡度。如果强行限制为 3DoF，会把真实姿态变化压成错误的平面约束。只有严格平整地面、传感器姿态几乎不变化，并且明确只做二维导航时，才考虑设为 `true`。

另外，四足步态可能在瞬间满足低速度条件，应关注 `TryZUPT=true` 是否产生误静止判断。如果装机后出现与落脚节奏同步的跳变，可以做一轮 `TryZUPT=false` 的对照实验。

## 8. 点云飞点为什么减少，以及还能怎么优化

### 8.1 当前生效的低噪点配置

当前改用较保守的 StereoBM：

```text
Stereo/DenseStrategy             = 0
StereoBM/BlockSize               = 15
StereoBM/NumDisparities          = 96
StereoBM/UniquenessRatio         = 20
StereoBM/TextureThreshold        = 10
StereoBM/SpeckleWindowSize       = 200
StereoBM/SpeckleRange            = 2
StereoBM/Disp12MaxDiff           = 1
```

点云过滤：

```text
Grid/RangeMin                    = 0.3 m
Grid/RangeMax                    = 3.0 m
Grid/CellSize                    = 0.05 m
Grid/DepthDecimation             = 4
Grid/NoiseFilteringRadius        = 0.1 m
Grid/NoiseFilteringMinNeighbors  = 5
Grid/MinClusterSize              = 10
```

这些参数位于：

```text
/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/config/
rtabmap_openvins_mapping_params.yaml
```

它们不是相机驱动参数，也不是 OpenVINS 参数。

### 8.2 为什么纯双目配置的飞点反而少

此前较激进的 SGBM 会在弱纹理、反光、重复纹理和鱼眼边缘区域尝试生成更多稠密视差。点更多，但错误匹配也更多。StereoBM 使用更大的块和更严格的唯一性/纹理阈值，在不可靠区域倾向于不输出深度，所以结果更稀疏但更干净。

这说明点云质量不是“越密越好”。对约 49.4 mm 的小基线双目，相机距离越远，视差越小，深度误差会快速放大。将 `Grid/RangeMax` 限制到 2.5～3.0 m 是合理取舍。

### 8.3 进一步优化顺序

如果仍有飞点，建议一次只改一组：

1. 先把 `Grid/RangeMax` 从 `3.0` 降到 `2.5`。
2. 将离群点过滤改为 `0.12 m / 8 neighbors`。
3. 将 `Grid/MinClusterSize` 从 `10` 提到 `20`。
4. 为左右目分别制作鱼眼有效区域 mask，屏蔽黑边、极端畸变边缘和机身遮挡。
5. 再根据场景考虑调整 StereoBM，而不是一开始就换回更密的 SGBM。

这些优化会删掉一部分真实的细小物体或远处点，因此应通过同一场景 A/B 对比决定，不要一次全部叠加。

## 9. RTAB-Map 源码编译问题

### 9.1 `matd_destroy` 链接错误

编译 RTAB-Map 0.23.8 时出现：

```text
librtabmap_core.so.0.23.8: undefined reference to `matd_destroy'
```

错误发生在链接 `rtabmap-camera`、`rtabmap-console`、`rtabmap-stereoEval` 等工具时，来源是 AprilTag 相关符号没有被正确解析，而不是这些工具本身的源码错误。

当前处理方式是关闭 RTAB-Map 内置 AprilTag，同时保留 OpenVINS、图优化器、Qt 应用和常用工具：

```bash
cd ~/rtabmap_humble_ws
source /opt/ros/humble/setup.bash
source ~/robot_ws/openvins_ws/install/setup.bash

MAKEFLAGS="-j4" colcon build \
  --symlink-install \
  --executor sequential \
  --event-handlers console_direct+ \
  --cmake-force-configure \
  --packages-select rtabmap \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DWITH_OPENVINS=ON \
    -DWITH_APRILTAG=OFF \
    -DWITH_QT=ON \
    -DBUILD_APP=ON \
    -DBUILD_TOOLS=ON
```

说明：

- `MAKEFLAGS="-j4"` 限制单个 Make/CMake 包最多使用 4 个并行任务。
- `--executor sequential` 让 colcon 一次构建一个 ROS 包，但包内部仍可 `-j4`。
- `--cmake-force-configure` 强制重新执行 CMake，使 `WITH_APRILTAG=OFF` 真正写入配置。
- 只写 `-DWITH_APRILTAG=OFF` 可能会沿用旧 CMake 缓存中的其他选项；为了可复现，完整命令应明确写出重要功能开关。

当前 `rtabmap --version` 已确认包含：

```text
OpenCV, PCL, Python3, TORO, g2o, GTSAM, Vertigo, Ceres,
OpenNI/OpenNI2, RealSense2, libpointmatcher, OctoMap, OpenVINS
```

AprilTag 当前关闭。除非确实需要 AprilTag 检测功能，否则不影响本项目的普通 ORB 回环和 OpenVINS。

### 9.2 为什么还要重新编译 `rtabmap_ros`

RTAB-Map 核心库重新编译后，`rtabmap_odom`、`rtabmap_slam` 和 `rtabmap_viz` 都要链接新的核心库。尤其是修改了功能开关、版本或 ABI 时，应继续构建 ROS 包：

```bash
cd ~/rtabmap_humble_ws
source /opt/ros/humble/setup.bash
source ~/robot_ws/openvins_ws/install/setup.bash
source ~/rtabmap_humble_ws/install/setup.bash

MAKEFLAGS="-j4" colcon build \
  --symlink-install \
  --executor sequential \
  --event-handlers console_direct+ \
  --packages-up-to rtabmap_odom rtabmap_slam rtabmap_viz \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

验证当前 shell 使用的是刚编译的版本：

```bash
source ~/rtabmap_humble_ws/install/setup.bash
which rtabmap
rtabmap --version
ros2 pkg prefix rtabmap_odom
ros2 pkg prefix rtabmap_slam
```

如果终端 source 顺序错误，系统可能仍加载 `/opt/ros/humble` 或另一个工作空间中的旧库，表现为“明明编译成功，运行参数却没变化”。

### 9.3 修改 launch/YAML 后是否必须重新编译

使用 `--symlink-install` 时，launch 和 YAML 通常直接链接到源码目录，修改后不一定需要重新编译。可这样确认：

```bash
readlink -f ~/robot_ws/camera/install/stereo_camera_pkg_py/share/\
stereo_camera_pkg_py/launch/equi_imu_rtabmap.launch.py
```

如果结果指向 `src` 中的文件，保存后重新启动即可。如果修改了 C++、Python 安装规则、`setup.py`、`CMakeLists.txt`、`package.xml`，或者安装目录不是符号链接，则需要重编相关包。

## 10. 为什么每次启动又从零开始

最外层启动文件当前默认：

```text
delete_db_on_start = true
```

因此每次启动都会删除旧数据库并新建地图，从零开始是预期行为。继续原地图时使用相同数据库路径并关闭删除：

```bash
ros2 launch stereo_camera_pkg_py equi_imu_rtabmap.launch.py \
  database_path:=$HOME/.ros/rtabmap_openvins_equi_mapping.db \
  delete_db_on_start:=false
```

三种典型模式：

| 目的 | `delete_db_on_start` | `Mem/IncrementalMemory` | `Mem/LocalizationReadOnly` |
| --- | --- | --- | --- |
| 全新建图 | true | true | false |
| 继续扩展旧地图 | false | true | false |
| 只在旧地图定位 | false | false | true |

切换模式前先备份 `.db`。继续建图不等于一定能立即恢复原位置，系统仍需要看到足够相似的场景并成功重定位。

## 11. 漂移和回环排查结果

### 11.1 前一次明显漂移的运行

前一次约 18:16 的运行特征：

- 约 57 个地图节点，轨迹约 16.2 m。
- 多个连续节点的视觉活跃特征降到 0。
- 线性标准差大于 5 cm 的状态累计约 13.5 s，最大约 0.247 m。
- RTAB-Map 找到了外观相似候选，但加入图后与已有轨迹产生很大的角度冲突。
- 图优化误差比远大于 `RGBD/OptimizeMaxError=3`，因此候选回环被拒绝。

这不是简单的“回环阈值太严格”，而是回环候选和已经漂移的局部轨迹在几何上不一致。直接放宽阈值可能把错误约束写进地图。

### 11.2 最新一次较好的运行

最新一次约 18:41 的运行：

```text
运行时长                 ≈ 123 s
地图节点                 = 115
轨迹长度                 ≈ 9.1 m
相机频率                 ≈ 20.02～20.04 Hz
相机丢帧                 = 0
IMU 频率                  ≈ 199.93～199.97 Hz
IMU checksum/missing      = 0
OpenVINS 初始化时间       ≈ 4.93 s
初始化后线性 std 最大值  ≈ 0.0884 m
初始化后线性 std P95     ≈ 0.0277 m
```

数据库统计还显示：

- 有实际接受的全局回环假设，也有少量被拒绝的候选。
- 最终 `map -> odom` 修正约为 `0.253 m / 0.851°`。
- 最大姿态修正约 `3.296°`。
- 原始里程计起终点距离约 `0.219 m`，图优化后约 `0.107 m`。

因此最新一次不是“完全没有误差”，但回环和图优化已经在工作，整体明显好于前一次。

### 11.3 最新一次仍存在的薄弱时段

最新数据库中仍有两段活跃特征明显偏低，大约位于：

```text
51.2～55.3 s
105.7～109.8 s
```

导出原图后可看到快速抬头/低头时有明显径向运动模糊。OpenVINS 在这些时段视觉更新变弱，主要依靠 IMU 传播；如果模糊持续更久或动作更猛烈，就可能再次发生不可恢复的漂移。

首要对照实验不是修改回环阈值，而是把曝光从 `580` 降到 `250～300`，沿同一路径做同样的抬头、低头和回到原位动作。

## 12. 以后自己怎么看日志

排查顺序建议固定为：

```text
相机/IMU 数据质量
        ↓
OpenVINS 是否初始化、特征是否下降、协方差是否增长
        ↓
RTAB-Map 是否找到候选、几何验证是否通过
        ↓
图优化是否接受约束、map→odom 是否发生修正
        ↓
导出图像和轨迹确认根因
```

不要一看到漂移就先改 RTAB-Map 回环参数。先判断漂移发生在传感器、VIO、回环检测还是图优化中的哪一层。

### 12.1 找到最新 launch 日志

ROS 2 启动时会打印类似：

```text
All log files can be found below
/home/bird/.ros/log/2026-07-17-18-41-05-...-368312
```

快速找到最近的运行目录：

```bash
latest_launch=$(find ~/.ros/log -mindepth 1 -maxdepth 1 -type d \
  -name '20*' -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-)
printf '%s\n' "$latest_launch"
sed -n '1,120p' "$latest_launch/launch.log"
```

`launch.log` 主要记录：

- 每个节点是否成功启动。
- 节点名称和 PID。
- 节点是否崩溃、退出码是多少。
- 是否是用户按 Ctrl-C 正常停止。

### 12.2 根据 PID 找节点日志

ROS 2 节点详细日志通常不在刚才的 launch 子目录内，而是直接放在 `~/.ros/log` 下，文件名格式为：

```text
节点名_PID_时间戳.log
```

例如 `launch.log` 中有：

```text
[stereo_odometry-7]: process started with pid [368325]
```

就可以找：

```bash
ls -lt ~/.ros/log/*_368325_*.log
```

也可以按节点找最近一个：

```bash
odom_log=$(ls -t ~/.ros/log/stereo_odometry_*.log | head -n 1)
rtabmap_log=$(ls -t ~/.ros/log/rtabmap_*.log | head -n 1)
camera_log=$(ls -t ~/.ros/log/stereo_v4l2_direct_node_*.log | head -n 1)
imu_log=$(ls -t ~/.ros/log/wit_imu_node_*.log | head -n 1)

printf '%s\n' "$odom_log" "$rtabmap_log" "$camera_log" "$imu_log"
```

### 12.3 先检查相机

```bash
rg -n -i \
  'configured|camera control|direct capture|fps|drop|stall|timeout|reconnect|warn|error' \
  "$camera_log"
```

正常参考：

- 实际频率稳定在约 20 Hz。
- `drop=0` 或不持续增长。
- 没有 `stall`、`timeout`、`reconnect`。
- 日志中的曝光、增益、锐度确实是本次实验设置。

频率正常不代表画面一定正常。运动模糊、过曝、欠曝、反光和自动参数变化都需要看实际图像。

### 12.4 再检查 IMU

```bash
rg -n -i \
  'imu:|hz|checksum|missing|resync|phase|warn|error' \
  "$imu_log"
```

正常参考：

- 频率约 200 Hz。
- `checksum=0`、`missing=0`、`resync=0`。
- 相机/IMU 相位误差稳定，不出现突然跳变。

如果 IMU 丢包、时间戳倒退或相机时间偏移方向错误，快速旋转时通常最先暴露问题。

### 12.5 看 OpenVINS 标准差

当前 `stereo_odometry` 日志中比较有用的是：

```text
std dev=<线性标准差>m|<角度标准差>rad
```

经验参考，不是硬性判定：

| 指标 | 良好 | 注意 | 严重 |
| --- | --- | --- | --- |
| 线性 std | `< 0.02 m` | `0.02～0.05 m` | 持续 `> 0.05 m`，尤其 `> 0.1 m` |
| 角度 std | `< 0.05 rad` | `0.05～0.2 rad` | 持续 `> 0.2 rad` |

启动前的 `0/0` 和初始化附近的 `1/1` 可能是状态占位值，统计时要排除。快速汇总：

```bash
sed -nE 's/.*std dev=([0-9.]+)m\|([0-9.]+)rad.*/\1 \2/p' "$odom_log" |
awk '
  $1 > 0 && $1 < 0.99 {
    n++;
    if ($1 > max_l) max_l=$1;
    if ($2 > max_a) max_a=$2;
    if ($1 > 0.05) bad_l++;
    if ($2 > 0.20) bad_a++;
  }
  END {
    printf("samples=%d max_linear=%.6f max_angular=%.6f linear_gt_5cm=%d angular_gt_0.2rad=%d\n",
           n, max_l, max_a, bad_l, bad_a);
  }'
```

当前集成里的 `quality=0` 不能直接当成“跟踪失败”，应主要看特征数、标准差、图像和轨迹是否连续。

### 12.6 运行时实时观察

```bash
ros2 topic hz /cam0/image_rect
ros2 topic hz /cam1/image_rect
ros2 topic hz /imu/data_raw
ros2 topic echo /odom_info --field features
ros2 topic echo /rtabmap/info --field loop_closure_id
ros2 topic echo /rtabmap/info --field proximity_detection_id
ros2 run tf2_ros tf2_echo map odom
```

也可以画特征数曲线：

```bash
rqt_plot /odom_info/features
```

OpenVINS 活跃特征数可粗略这样看：

| 特征数 | 解释 |
| ---: | --- |
| `> 30` | 通常良好 |
| `15～30` | 可用，但应结合运动速度看 |
| `5～15` | 危险，连续出现容易漂移 |
| `0` | 该时刻没有有效视觉更新，主要依赖 IMU 传播 |

偶尔一帧为 0 不一定出问题；连续几秒为 0 才是真正危险。

### 12.7 从数据库看回环是否真正生效

文本日志适合看传感器频率、初始化、延迟和协方差；回环候选、接受/拒绝原因、图优化修正和节点统计应优先看 RTAB-Map 数据库。

先加载正确工作空间：

```bash
source /opt/ros/humble/setup.bash
source ~/rtabmap_humble_ws/install/setup.bash
```

生成统计：

```bash
db=~/.ros/rtabmap_openvins_equi_mapping.db
QT_QPA_PLATFORM=offscreen rtabmap-report --stats "$db" | less
```

筛选关键字段：

```bash
QT_QPA_PLATFORM=offscreen rtabmap-report --stats "$db" |
rg -i 'loop|hypothesis|inliers|rejected|optimization|map.*odom|feature'
```

重点理解：

- `Accepted_hypothesis` 非零：全局回环假设通过了检测和几何验证。
- `Rejected=1`：候选被拒绝，要继续看内点或图优化误差。
- `Visual_inliers >= Vis/MinInliers`：只是几何验证的必要条件，不保证图一致性一定通过。
- 优化误差比超过 `RGBD/OptimizeMaxError`：候选与当前图冲突，被拒绝通常是正确行为。
- `MapToOdom` 发生变化：RTAB-Map 确实对局部里程计进行了全局修正。
- 报告中的 `loops=` 可能包含全局回环和邻近检测连接，不能只靠一个总数下结论。

### 12.8 导出数据库图像，确认是否运动模糊

```bash
db=~/.ros/rtabmap_openvins_equi_mapping.db
image_dir=$(mktemp -d /tmp/rtabmap_images.XXXXXX)
rtabmap-export --images_id --output_dir "$image_dir" "$db"
printf '%s\n' "$image_dir"
eog "$image_dir"
```

观察内容：

- 快速抬头、低头时是否有明显拉丝或径向模糊。
- 鱼眼黑边和机身是否占据大量画面。
- 左右目亮度是否一致。
- 是否有大面积白墙、地板、玻璃或重复纹理。
- 是否有行人等动态目标主导特征。
- 左右目同一时刻是否同步。

### 12.9 导出优化前后轨迹

```bash
db=~/.ros/rtabmap_openvins_equi_mapping.db
QT_QPA_PLATFORM=offscreen rtabmap-report --poses_raw "$db"
```

通常会生成：

```text
*_odom.txt   原始里程计轨迹
*_slam.txt   回环和图优化后的轨迹
```

对比两条轨迹可以回答：

- OpenVINS 本身漂了多少。
- RTAB-Map 是否产生修正。
- 修正发生在哪个时间段。
- 图优化是否让起终点更接近。

只有实验路径确实回到起点时，“起终点距离”才有闭环误差意义；没有回到原位时不能用它判断好坏。

## 13. 参数是否真正生效的检查方法

不要只看源码里的 YAML，要确认运行节点最终接收到的参数。

```bash
ros2 param get /openvins_stereo_odometry Odom/Strategy
ros2 param get /openvins_stereo_odometry OdomOpenVINS/NumPts
ros2 param get /rtabmap Reg/Force3DoF
ros2 param get /rtabmap Grid/RangeMax
ros2 param get /rtabmap Grid/NoiseFilteringRadius
ros2 param get /rtabmap Stereo/DenseStrategy
```

也可以在启动日志中搜索 RTAB-Map 打印的参数覆盖：

```bash
rg -n 'Setting RTAB-Map parameter|Grid/RangeMax|DenseStrategy|Force3DoF' \
  "$rtabmap_log"
```

YAML 或 Python 字典中如果同一个 key 写了两次，后面的值会覆盖前面的值。此前参数中同时写过两组：

```text
Grid/NoiseFilteringRadius       = 0.15，后来又写 0.1
Grid/NoiseFilteringMinNeighbors = 8，后来又写 5
```

实际只会保留最后的 `0.1/5`。因此每个参数只保留一处，避免以为两组都生效。

## 14. 建议的标准测试流程

为了让每次测试可以比较，建议固定流程：

1. 固定场地、光线和行走路线。
2. 启动后静止 5～10 秒，让 OpenVINS 完成初始化。
3. 先缓慢平移和转动，再做抬头/低头。
4. 中途经过有纹理区域，避免一直对着白墙或地板。
5. 回到起点并停留 5 秒，给回环检测留时间。
6. Ctrl-C 正常结束，等待数据库写完。
7. 记录本次曝光、增益、参数文件和数据库名。
8. 按“传感器 → OpenVINS → RTAB-Map → 图像/轨迹”的顺序分析。
9. 每轮只改一个主要变量。

建议为每轮实验单独命名数据库：

```bash
ros2 launch stereo_camera_pkg_py equi_imu_rtabmap.launch.py \
  database_path:=$HOME/.ros/rtabmap_test_exposure280.db \
  delete_db_on_start:=true
```

这样不会覆盖上一轮，也便于把日志、图像和数据库对应起来。

## 15. 下一步建议

按优先级排列：

1. 将曝光从 `580` 降至 `250～300`，复现快速抬头/低头测试。
2. 保持 RTAB-Map 回环和图优化阈值不变，先确认 OpenVINS 特征低谷是否明显减少。
3. 为左右鱼眼图像制作各自的有效区域 mask，屏蔽黑边和机身。
4. 装到机器狗后准确测量并发布 `base_link -> imu_link`，保持 `Reg/Force3DoF=false`。
5. 对四足步态分别测试 `TryZUPT=true/false`，检查是否有与落脚节奏同步的误修正。
6. 点云仍有飞点时，按 `RangeMax 2.5 m → 0.12/8 离群过滤 → MinClusterSize 20` 的顺序逐项对比。

## 16. 一页式检查清单

启动前：

- [ ] source 顺序正确，`rtabmap --version` 含 OpenVINS。
- [ ] 左右目均发布 `equidistant` `CameraInfo`。
- [ ] `imu_link -> cam0/cam1` 是本次联合标定外参。
- [ ] 相机时间偏移为约 `+30.7304 ms`，方向符合 `t_imu=t_cam+shift`。
- [ ] 手动曝光、固定白平衡、固定焦距。
- [ ] 明确本轮是新建图、继续建图还是定位模式。

运行中：

- [ ] 相机约 20 Hz，IMU 约 200 Hz。
- [ ] 无相机丢帧、重连和 IMU 丢包。
- [ ] `/odom_info/features` 大部分时间高于 15～30。
- [ ] 快速动作时图像没有严重拉丝。
- [ ] RViz `Fixed Frame` 使用 `map`。

结束后：

- [ ] 根据 PID 找到相机、IMU、里程计和 RTAB-Map 节点日志。
- [ ] 检查 OpenVINS 线性/角度标准差是否持续增长。
- [ ] 用 `rtabmap-report --stats` 看候选回环的接受、拒绝和图优化误差。
- [ ] 导出数据库图像检查模糊、黑边和弱纹理。
- [ ] 对比 `_odom.txt` 与 `_slam.txt`，确认回环是否真的修正轨迹。

---

本文记录的是 2026-07-17 当前已验证配置。以后更换相机分辨率、镜头焦距、左右目顺序、IMU 安装位置、同步方式或机械结构后，旧的内参、外参和时间偏移都不应直接沿用，应重新标定并重新核对完整 TF 链。
