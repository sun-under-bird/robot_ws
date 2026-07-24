# 2026-07-22 D435i、HB 双目、OpenVINS 与 RTAB-Map 工作总结

本文记录 2026 年 7 月 22 日围绕 D435i 对照测试、HB USB 双目曝光优化、高分辨率重新标定、OpenVINS/RTAB-Map 接入，以及快速运动漂移问题所完成的工作和得到的结论。

## 1. 当日工作概览

当天工作分为四条主线：

1. 使用 D435i 对照测试 OpenVINS，并重新标定 D435i 相机—IMU 外参。
2. 排查并解决 HB USB 双目自动曝光、启动变暗和周期性闪烁问题。
3. 完成 HB 双目每目 `1280×720` 高分辨率标定、联合标定和建图启动文件接入。
4. 从运行日志、RTAB-Map 数据库和 OpenVINS/RTAB-Map 源码定位快速运动漂移及回环不能修正的原因。

最终确认：当前主要问题不是 USB 丢帧、IMU 丢包、CPU 性能或 RTAB-Map 回环识别失败，而是快速运动时 OpenVINS 的 KLT 连续跟踪失效。视觉更新消失后，滤波器继续依赖 IMU 积分传播，位置和速度快速发散；重新看到旧场景时，RTAB-Map 虽能识别回环，但新旧约束冲突过大，因此拒绝将回环加入图中。

## 2. 主要文件索引

| 用途 | 文件 |
| --- | --- |
| HB 低分辨率 OpenVINS 建图启动 | `/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/launch/hb_imu_rtabmap.launch.py` |
| HB 高分辨率 OpenVINS 建图启动 | `/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/launch/hb_highres_imu_rtabmap.launch.py` |
| OpenVINS 与 RTAB-Map 通用启动 | `/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/launch/rtabmap_openvins_stereo_mapping.launch.py` |
| OpenVINS 与 RTAB-Map 参数 | `/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/config/rtabmap_openvins_mapping_params.yaml` |
| D435i OpenVINS 参数 | `/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/config/d435i_rtabmap_openvins.yaml` |
| HB 低分辨率相机启动 | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/launch/usb_camera_openvins_15fps.launch.py` |
| HB 自动曝光相机启动 | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/launch/usb_camera_openvins_15fps_auto_exposure.launch.py` |
| HB 高分辨率相机启动 | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/launch/yuyv_100_to_50fps.launch.py` |
| HB 相机直采节点 | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/src/stereo_v4l2_direct_node.cpp` |
| HB 高分辨率左目 CameraInfo | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/config/left_hb_2560.yaml` |
| HB 高分辨率右目 CameraInfo | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/config/right_hb_2560.yaml` |
| D435i 联合标定结果 | `/home/bird/kalibr_data/d435i_ros1-results-imucam.txt` |
| HB 高分辨率双目标定结果 | `/home/bird/kalibr_data/stereo_calib-results-cam.txt` |
| HB 高分辨率联合标定结果 | `/home/bird/kalibr_data/cam_imu_repeat_01-results-imucam.txt` |
| 自动曝光专项总结 | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/docs/2026-07-22_usb_camera_auto_exposure_work_summary.md` |

## 3. D435i 对照测试与标定验证

### 3.1 D435i 运行表现

使用 D435i 运行 OpenVINS 后确认，它在较暗环境和快速运动中的视觉前端、速度估计和位置协方差明显比 HB 相机稳定。

D435i 的主要优势来自：

- 更适合运动视觉的成像链路；
- 相机和 IMU 硬件同步更加完整；
- 更高、更加稳定的实际图像帧率；
- 曝光和增益控制更适合快速运动；
- IMU 噪声和时间戳质量更好。

因此，D435i 快速运动时不容易漂，并不是因为 RTAB-Map 对它使用了特殊的回环修正，而是 OpenVINS 前端和惯性传播在进入 RTAB-Map 前就更加稳定。

### 3.2 D435i 联合标定结果

当天重新完成了一次 D435i 双目相机—IMU 联合标定。

| 项目 | 结果 |
| --- | ---: |
| cam0 重投影误差 | `0.2895 px` |
| cam1 重投影误差 | `0.2985 px` |
| 双目基线 | `49.9717 mm` |
| cam0 时间偏移 | `4.8444 ms` |
| cam1 时间偏移 | `4.8408 ms` |
| 陀螺仪残差 | `0.00482 rad/s` |
| 加速度计残差 | `0.09486 m/s²` |

标定出的双目基线、外参数量级和 D435i 官方参数相符，说明当前 Kalibr 操作流程没有明显的单位错误、矩阵方向错误或基线方向错误。

这项验证只能说明标定流程基本可信，不能证明 HB 相机的时间同步、曝光、机械刚性和 IMU 质量一定与 D435i 相同。

### 3.3 无 orientation 的 IMU 报错

运行 D435i 时 RTAB-Map 曾输出：

```text
IMU received doesn't have orientation set, it is ignored.
```

该报错来自 RTAB-Map 的异步 IMU 姿态接口。原始 IMU 消息只包含角速度和线加速度，没有填写 orientation，因此这一路消息被 RTAB-Map 忽略。

这不等于 OpenVINS 没有收到 IMU。OpenVINS 使用的是原始角速度和线加速度，仍可正常进行状态传播。

## 4. HB USB 双目自动曝光排查

### 4.1 遇到的问题

当天主要排查了以下现象：

- 启用自动曝光后画面先亮后暗；
- 软件自动曝光运行时周期性忽明忽暗；
- `stereo_v4l2_direct_node` 和 `v4l2_camera` 在同一相机上的亮度表现不同；
- 相机曝光参数已经设置，但图像仍然明显偏暗。

### 4.2 相机实际曝光控件

相机使用的主要 V4L2 控件为：

```text
auto_exposure
exposure_time_absolute
backlight_compensation
gain
```

当天确认的硬件范围和默认值：

| 控件 | 范围或枚举 | 默认值 |
| --- | --- | ---: |
| `auto_exposure` | `1` 手动，`3` Aperture Priority | `3` |
| `exposure_time_absolute` | `1～10000` | `156` |
| `backlight_compensation` | `0～100` | `48` |
| `gain` | `0～255` | `100` |

### 4.3 完成的代码修改

在 `stereo_v4l2_direct_node.cpp` 中增加了软件自动曝光功能，包括：

- 使用左右目中央区域灰度均值进行测光；
- 可配置目标亮度、曝光最小值和最大值；
- 可配置死区、更新周期和响应系数；
- 软件最大曝光设置为 `0` 时使用设备硬件最大值；
- USB 重连后重新初始化曝光状态；
- 启动时检查曝光模式和参数范围；
- 根据手动/自动曝光模式调整控件写入顺序。

增加的主要 ROS 参数：

```text
software_auto_exposure
software_auto_exposure_target
software_auto_exposure_min
software_auto_exposure_max
software_auto_exposure_deadband
software_auto_exposure_update_interval
software_auto_exposure_response
```

### 4.4 软件自动曝光闪烁的原因

软件闭环曾使用：

```text
target = 105
deadband = 5
update_interval = 10 帧
response = 0.25
```

运行中曝光值约在 `4831～5190` 之间变化，中央灰度约在 `90～122` 之间变化。画面呈现连续若干帧偏暗、随后若干帧偏亮的周期性波动。

根因是该相机写入曝光控件后，需要多帧才能完全反映到图像。软件闭环在上一条曝光指令尚未完全生效时再次调节，产生过冲和反向修正。

### 4.5 画面偏暗的真正原因

对照 `v4l2_camera` 后确认，画面过暗的主要原因不是分辨率，也不是图像拆分，而是背光补偿被设成了 `0`。

同一场景实测：

| 背光补偿 | 左目亮度均值 | 表现 |
| ---: | ---: | --- |
| `0` | `23.17` | 明显偏暗 |
| `48` | `95.16` | 亮度正常且稳定 |

### 4.6 最终采用的曝光方案

最终方案为：

```text
auto_exposure = 3
backlight_compensation = 48～50
exposure_time_absolute = 156
software_auto_exposure = false
```

连续 60 帧验证结果：

```text
亮度 mean = 97.65
亮度 min  = 97.55
亮度 max  = 97.75
亮度 std  = 0.06
相机发布频率约 14.3 FPS
内核序号丢帧为 0
```

结论是：对当前相机，稳定的硬件自动曝光和合理背光补偿优于响应较快的软件曝光闭环。

## 5. HB 高分辨率重新标定

### 5.1 高分辨率双目标定

完成每目 `1280×720` 的双目标定。

| 项目 | 结果 |
| --- | ---: |
| cam0 焦距 | `850.062 × 849.031` |
| cam1 焦距 | `838.811 × 837.262` |
| 双目基线 | `50.214 mm` |

双目标定使用 `pinhole-radtan` 模型，转换到 ROS CameraInfo 后使用 `plumb_bob`。

### 5.2 高分辨率联合标定

联合标定结果：

| 项目 | 结果 |
| --- | ---: |
| cam0 重投影误差 | `0.3633 px` |
| cam1 重投影误差 | `0.3450 px` |
| 陀螺仪残差 | `0.02942 rad/s` |
| 加速度计残差 | `0.12597 m/s²` |
| cam0 时间偏移 | `30.2176 ms` |
| cam1 时间偏移 | `30.2552 ms` |
| 两目平均时间偏移 | `30.2364 ms` |

高分辨率 `imu_link → cam0` 外参：

```text
translation = [-0.05336053, -0.02799721, -0.00939943]
quaternion  = [-0.503205891611, -0.470005149956,
                0.498764638674,  0.526415069001]
```

高分辨率 `imu_link → cam1` 外参：

```text
translation = [-0.05226179, 0.02220343, -0.00977532]
quaternion  = [-0.519478984559, -0.492300883370,
                0.480156411767,  0.507179697024]
```

### 5.3 高分辨率运行链路

新增 `hb_highres_imu_rtabmap.launch.py`，运行链路为：

```text
总图采集：2560×720 YUYV，设备配置 60 FPS
左右拆分：每目 1280×720 mono8
VIO 发布：20 FPS
IMU：约 200 Hz
OpenVINS：双目惯性里程计
RTAB-Map：双目建图和回环
```

高分辨率使用独立数据库：

```text
~/.ros/rtabmap_openvins_hb_highres_mapping.db
```

### 5.4 两套标定不能混用

当前同时存在两套 HB 标定：

| 链路 | 每目分辨率 | 时间偏移 | 启动文件 |
| --- | ---: | ---: | --- |
| 低分辨率 | `640×400` | 约 `40.381 ms` | `hb_imu_rtabmap.launch.py` |
| 高分辨率 | `1280×720` | 约 `30.236 ms` | `hb_highres_imu_rtabmap.launch.py` |

两套配置的内参、CameraInfo、相机—IMU 外参和时间偏移必须成套使用，不能把高分辨率标定值直接替换到低分辨率运行链路中。

## 6. 启动顺序调整

低分辨率和高分辨率启动文件都调整为：

```text
立即启动：相机、IMU、相机静态 TF、图像校正
等待 5 秒
同时启动：OpenVINS、RTAB-Map、rtabmap_viz
```

该修改的目的是在 OpenVINS 启动前先让相机、IMU、串口时间戳和图像话题稳定。

当天还尝试过：

1. 让 RTAB-Map 直接订阅 `/odom`，读取 OpenVINS 实际协方差；
2. 将 RTAB-Map 建图进一步延后到 OpenVINS 初始化完成之后。

后续按要求撤回了这部分修改。当前状态为：

- 传感器启动后等待 5 秒；
- OpenVINS 和 RTAB-Map 同时启动；
- RTAB-Map 设置 `odom_frame_id="odom"`，通过 TF 读取里程计位姿；
- RTAB-Map继续订阅 `/odom_info` 获取特征统计。

## 7. OpenVINS 参数检查

当日最终使用的主要 OpenVINS 参数：

```text
Odom/Strategy                         = 10
OdomOpenVINS/UseStereo                = true
OdomOpenVINS/UseKLT                   = true
OdomOpenVINS/NumPts                   = 500
OdomOpenVINS/MinPxDist                = 10
FAST/Threshold                        = 10
OdomOpenVINS/MaxClones                = 15
OdomOpenVINS/MaxSLAM                  = 50
OdomOpenVINS/MaxSLAMInUpdate          = 30
OdomOpenVINS/MaxMSCKFInUpdate         = 80
OdomOpenVINS/TryZUPT                  = true
OdomOpenVINS/ZUPTChi2Multiplier       = 0.5
OdomOpenVINS/ZUPTMaxVelodicy          = 0.1
OdomOpenVINS/ZUPTMaxDisparity         = 0.5
OdomOpenVINS/ZUPTOnlyAtBeginning      = false
```

IMU 噪声参数：

```text
AccelerometerNoiseDensity = 0.02
AccelerometerRandomWalk   = 0.002
GyroscopeNoiseDensity     = 0.0015
GyroscopeRandomWalk       = 0.00015
```

这些参数会影响滤波器对 IMU 和视觉测量的权重以及协方差增长速度，但无法在视觉完全失效时为系统重新产生视觉约束。

## 8. 最新一次严重漂移的日志结论

重点分析了 20:05:53 的低分辨率运行：

```text
/home/bird/.ros/log/2026-07-22-20-05-53-090051-bird-HP-Laptop-15-fd1xxx-240300
```

对应数据库：

```text
/home/bird/.ros/rtabmap_openvins_hb_mapping.db
```

### 8.1 传感器链路正常

相机：

```text
每目分辨率：640×400
配置帧率：15 FPS
实际采集/发布：约 14.30～14.31 FPS
内核序号丢帧：0
```

IMU：

```text
实际频率：约 199.3～199.6 Hz
持续校验错误：0
持续缺失样本：0
时间戳重同步：0
```

OpenVINS：

```text
最大单帧更新时间约 55.9 ms
最大输出延迟约 94.7 ms
没有持续处理积压
没有 image out-of-order 警告
```

因此可以排除 USB 持续丢帧、IMU 持续丢包和 CPU 长时间算不过来。

### 8.2 漂移发生的时间线

严重漂移开始于运行末尾约 7 秒：

| 节点 | OpenVINS 特征/本地图点 | 估计速度 | 线位置标准差 | RTAB-Map 回环结果 |
| ---: | ---: | ---: | ---: | --- |
| 88 | `20 / 54` | `0.294 m/s` | `0.016 m` | 接受节点 72，`72/119` 内点 |
| 89 | `0 / 81` | `0.649 m/s` | `0.029 m` | 接受节点 76，`100/157` 内点 |
| 90 | `6 / 111` | `1.143 m/s` | `0.043 m` | 候选 72，`0/48`，拒绝 |
| 91 | `5 / 110` | `1.851 m/s` | `0.065 m` | `72/112` 内点，角度冲突 `158.9°` |
| 92 | `0 / 96` | `1.811 m/s` | `0.090 m` | `267/368` 内点，角度冲突 `172.8°` |
| 93 | `0 / 60` | `1.985 m/s` | `0.122 m` | `283/370` 内点，角度冲突 `134.6°` |
| 94 | `1 / 29` | `2.044 m/s` | `0.142 m` | `279/384` 内点，角度冲突 `134.1°` |
| 95 | `0 / 6` | `2.178 m/s` | `0.176 m` | `276/369` 内点，角度冲突 `83.2°` |

最终 OpenVINS 位姿约为：

```text
x =  9.601 m
y = -2.641 m
z =  0.767 m
距原点约 9.99 m
```

从节点 88 到节点 95，约 7.34 秒内产生了约 `10.14 m` 的错误位移。

## 9. OpenVINS 前端失效原因

### 9.1 KLT 连续跟踪机制

当前 OpenVINS KLT 固定使用：

```text
金字塔层数：5
光流窗口：15×15
```

新帧关键点的初值直接使用上一帧像素位置，然后调用 `calcOpticalFlowPyrLK()`。当前实现没有根据 IMU 角速度先预测特征在新图像中的位置。

快速转动时容易出现：

- 帧间特征位移过大；
- 运动模糊；
- 大量旧特征离开视野；
- RANSAC 将不一致的光流结果剔除。

这会导致能够参与 MSCKF/SLAM 更新的有效特征快速下降。

### 9.2 日志中的 Features 含义

RTAB-Map OpenVINS 封装中的特征统计为：

```text
features_SLAM.size() + good_features_MSCKF.size()
```

因此数据库中的 `Odometry/Features=0` 表示当前没有形成有效的 SLAM/MSCKF 更新特征，不一定表示 FAST 在原始图像中一个角点都没有检测到。

日志中的 `quality=0` 也不能单独当作 OpenVINS 跟踪失败。ROS 日志打印的是 `info.reg.inliers`，而 OpenVINS 封装没有正常填充该字段。判断 OpenVINS 应优先看：

```text
Odometry/Features
Odometry/LocalMapSize
线性和角度标准差
估计速度
原始图像质量
```

### 9.3 为什么重新看到场景也不会恢复

KLT 跟踪丢失后会重新检测角点，但新角点会得到新的特征 ID，只能形成新的短期局部轨迹。

OpenVINS 本身没有：

- 词袋场景检索；
- 旧地图特征搜索；
- 全局 PnP 重定位；
- 将 RTAB-Map 回环写回滤波器状态的接口。

所以重新看到电脑、桌子和人物，并不会自动修正已经错误的位置、速度、IMU bias 和滑动窗口状态。

## 10. 为什么失效后仍继续输出错误里程计

OpenVINS 初始化后，每个图像时刻都会先通过 IMU 传播状态并增加 clone。当前源码只有在 `is_initialized_vio=false` 时才会进入初始化，不会因为后续特征数量过低自动回到未初始化状态。

RTAB-Map 的 OpenVINS 封装只要能从 OpenVINS 状态中获得非单位位姿，就会继续计算并返回增量变换：

```text
t = previousPoseInv × currentPose
```

它没有检查：

- 有效更新特征是否为 0；
- LocalMap 是否持续下降；
- 协方差是否持续增长；
- 速度是否与实际运动不一致。

因此，即使视觉已经失效，它仍返回非空的 IMU 积分位姿。RTAB-Map 的 `Odom/ResetCountdown` 只有在里程计连续返回空变换时才会触发，所以简单把该参数从 `0` 改成其他值也无法可靠解决当前问题。

## 11. 为什么 ZUPT 没有阻止漂移

当前 ZUPT 参数：

```text
TryZUPT             = true
ZUPTChi2Multiplier  = 0.5
ZUPTMaxVelodicy     = 0.1 m/s
ZUPTMaxDisparity    = 0.5 px
ZUPTOnlyAtBeginning = false
```

OpenVINS 接受 ZUPT 需要满足：

1. 图像平均视差小于阈值并且有超过 20 个跟踪特征；或
2. 卡方检验通过并且当前估计速度小于 `0.1 m/s`。

故障节点 89 时已经出现：

```text
有效更新特征 = 0
估计速度 = 0.649 m/s
```

两条条件均不满足。错误速度继续增长后，ZUPT 更不可能通过，因此它不能作为视觉丢失后的通用 IMU 漂移制动器。

## 12. 为什么 RTAB-Map 回环没有修正轨迹

重新看到原场景后，RTAB-Map 获得了约 `267～283` 个视觉内点，说明回环识别和返回画面质量都不是主要问题。

当前：

```text
RGBD/OptimizeMaxError = 3.0
```

RTAB-Map 会在加入回环后计算图中每条边的绝对误差/标准差比例。如果线性或角度误差比例超过 3，就删除本次加入的回环。

这次回环与已经发散的 odom 约束产生了 `83°～173°` 的角度冲突，因此拒绝回环是正确的安全行为。

不建议通过增大 `RGBD/OptimizeMaxError` 或将其设为 0 强行接受这些回环，否则可能把整张地图拉坏。

## 13. 发现的两个结构问题

### 13.1 矫正图像坐标轴与相机外参不完全一致

当前 OpenVINS 输入为：

```text
/cam0/image_rect
/cam1/image_rect
Rtabmap/ImagesAlreadyRectified = true
```

低分辨率左右 CameraInfo 中的 rectification 旋转约为：

```text
左目：3.097°
右目：2.947°
```

当前静态 TF 发布的是原始相机光学坐标系外参。OpenVINS 封装在 `ImagesAlreadyRectified=true` 时使用投影矩阵 `P` 和零畸变，但没有把 CameraInfo 的 rectification `R` 合入相机—IMU 外参。

这相当于使用矫正图像的像素射线，却仍使用原始相机坐标轴外参，存在约 `3°` 的确定性坐标不一致。

它不足以单独产生超过 `100°` 的发散，但会降低高速旋转时视觉残差和 IMU 预测的一致性，应优先修正。

双目基线本身是正确的：

```text
静态 TF 两相机距离 = 0.0501910609 m
CameraInfo P 基线   = 0.0501910591 m
```

### 13.2 RTAB-Map 未使用 OpenVINS 实际协方差

当前 RTAB-Map 设置：

```text
odom_frame_id = "odom"
```

设置 `odom_frame_id` 后，RTAB-Map 会通过 TF 获取里程计并强制关闭 `/odom` 订阅。TF 只携带位姿，不包含速度和协方差。

因此数据库中的里程计方差一直为默认值：

```text
Memory/Odometry_variance_lin = 0.001
Memory/Odometry_variance_ang = 0.001
```

而故障期间 OpenVINS 实际线位置标准差已经从约 `0.016 m` 增长到约 `0.176 m`。

这不是 OpenVINS 发散的直接原因，也不足以让 `83°～173°` 的错误回环变得可接受，但会让 RTAB-Map 后端对已经变差的 odom 过度自信。

## 14. 当日最终结论

### 14.1 已经排除

- 相机持续掉帧；
- IMU 持续丢包；
- CPU 长时间算不过来；
- OpenVINS 初始化失败；
- 双目基线明显写错；
- RTAB-Map 没有识别出旧场景；
- 单纯由 IMU 噪声参数造成的瞬间发散。

### 14.2 已经确认

1. 快速运动时 KLT 连续跟踪失效是本次 VIO 发散的直接触发点。
2. 视觉更新消失后，OpenVINS 仍继续进行 IMU 传播并输出非空里程计。
3. 当前 OpenVINS 和 RTAB-Map 封装没有可靠的健康检测、自动重置和全局重定位机制。
4. RTAB-Map 可以重新识别旧场景，但会拒绝与错误 odom 严重冲突的回环。
5. 矫正图像坐标轴和原始相机外参之间存在约 `3°` 的接入不一致。
6. RTAB-Map 通过 TF 获取 odom 时没有使用 OpenVINS 的实际协方差。

## 15. 后续工作优先级

建议按以下顺序继续：

1. 为矫正后的左右图建立正确的 `cam0_rect/cam1_rect` 坐标系，并将 CameraInfo 的 `R` 合入相机—IMU 外参；或者让 OpenVINS 直接处理原始图像和原始 radtan 模型。
2. 提高实际 VIO 图像帧率，缩短运动曝光，并检查快速转动中间帧是否存在明显拉丝。
3. 对 `UseKLT=true/false` 进行相同轨迹、相同曝光、相同帧率的 A/B 测试。
4. 在 OpenVINS 封装或外部监控节点中增加健康状态机，至少联合判断有效特征、LocalMap、协方差和异常速度。
5. 进入 LOST 后停止写入错误里程计边，保持最后可信位姿，重置 OpenVINS，并由 RTAB-Map 创建新子图。
6. OpenVINS 恢复后通过 RTAB-Map 回环将新子图重新连接到旧地图。
7. 在 VIO 前端稳定后，再让 RTAB-Map 直接订阅 `/odom` 并验证实际协方差是否正确进入数据库。
8. 不要优先关闭 `RGBD/OptimizeMaxError`、盲目增大特征数或反复修改 IMU 噪声；这些操作不能解决本次失效机制。

## 16. 实验记录建议

后续每次测试应固定记录：

```text
启动文件
分辨率和实际 FPS
CameraInfo 版本
相机—IMU 外参版本
camera_time_offset_ms
曝光、增益、背光补偿和自动曝光模式
OpenVINS 参数文件
ROS launch 日志目录
RTAB-Map 数据库路径
是否发生快速转动、遮挡、暗场和返回旧场景
```

分析顺序建议固定为：

```text
相机频率/丢帧/曝光
→ IMU 频率/丢包/时间戳
→ OpenVINS Features/LocalMap/协方差/速度
→ RTAB-Map 回环候选/视觉内点/拒绝原因
→ odom 与 slam 轨迹对比
→ 必要时回到源码确认参数语义
```
