# 2026-07-21 新 USB 双目、OpenVINS 与 RTAB-Map 工作总结和排障手册

本文记录 2026-07-21 对新 USB 双目相机（下文简称 HB 双目）、WIT IMU、OpenVINS 和 RTAB-Map 的标定接入、运行测试、漂移分析和源码核查结果。后半部分整理为可重复使用的日志排查手册，目标是以后遇到“轨迹飘、回环不修正、重新看到场景也不恢复”时，能够快速判断问题处于相机、IMU、VIO、回环还是图优化层。

## 1. 今日核心结论

当前数据链路已经跑通：

```text
HB USB 双目 640×400/目，约 14.3 Hz ─┐
                                      ├─ RTAB-Map OpenVINS ─ odom → imu_link
WIT IMU，约 199.5 Hz ─────────────────┘
                                                  │
校正后的左右图 ───────────────────────────────────┴─ RTAB-Map ─ map → odom
```

今日得到的主要结论如下：

1. 最新联合标定的相机重投影误差约为 `0.275～0.278 px`，双目基线约 `50.19 mm`，相机外参与时间偏移已经写入 HB 启动文件。
2. 相机和 IMU 的发布频率、时间戳连续性和 USB 数据链路总体正常，今日几次严重漂移都不是由持续丢帧、IMU 丢包或算力不足直接引起的。
3. OpenVINS 在正常纹理和正常运动下可以维持 `20～60` 个有效更新特征，平移标准差约 `1～2 cm`，说明相机并非完全不可用。
4. 严重漂移通常由以下组合触发：长时间弱纹理、近距离黑色显示器占满视野、纯旋转或大幅转动、曝光过长、视觉特征更新持续为 0。
5. 重新看到桌子和电脑后，OpenVINS 的前端可以重新检测一部分角点，但这些新特征不一定能通过三角化和滤波卡方检验，因此不会自动把已经错误的位置和速度拉回来。
6. OpenVINS 是局部 VIO，不是带全局地图的重定位系统。当前 RTAB-Map 封装也不会在视觉失效时自动重置 OpenVINS。
7. RTAB-Map 可以在 `map` 层通过回环修正 `map → odom`，但不会把修正写回 OpenVINS 内部状态。已经发散的 `odom` 需要主动重置后再由 RTAB-Map 重定位到旧地图。
8. 切换到 RTAB-Map 内置 VINS-Fusion 可能改变正常阶段的鲁棒性，但不能直接获得重定位能力；当前 RTAB-Map 还以 `WITH_VINS_FUSION=OFF` 编译。
9. 当前最需要优先修正的是相机曝光与增益。19:21 的运行实际使用 `exposure_time_absolute=10000`、`gain=1`；写本文时源码已改为 `5000/128`，但曝光仍远大于注释所写的推荐值 `150`。

## 2. 当前主要文件

| 用途 | 文件 |
| --- | --- |
| HB 双目、IMU、TF、OpenVINS、RTAB-Map 一键启动 | `/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/launch/hb_imu_rtabmap.launch.py` |
| OpenVINS 和 RTAB-Map 参数 | `/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/config/rtabmap_openvins_mapping_params.yaml` |
| RTAB-Map OpenVINS 通用启动 | `/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/launch/rtabmap_openvins_stereo_mapping.launch.py` |
| HB 相机启动与 V4L2 控制参数 | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/launch/usb_camera_openvins_15fps.launch.py` |
| 左目 CameraInfo | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/config/left_hb.yaml` |
| 右目 CameraInfo | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/config/right_hb.yaml` |
| WIT IMU 启动 | `/home/bird/robot_ws/imu_ws/src/wit_imu/launch/wit_imu_new.launch.py` |
| 最新联合标定相机链 | `/home/bird/kalibr_data/cam_imu_repeat_01-camchain-imucam.yaml` |
| 最新联合标定结果 | `/home/bird/kalibr_data/cam_imu_repeat_01-results-imucam.txt` |
| 最新联合标定报告 | `/home/bird/kalibr_data/cam_imu_repeat_01-report-imucam.pdf` |
| 当前建图数据库 | `/home/bird/.ros/rtabmap_openvins_hb_mapping.db` |
| ROS 2 日志根目录 | `/home/bird/.ros/log` |

当前 HB 相机不是 7 月 17 日文档里的 `pinhole-equi` 鱼眼配置。最新联合标定使用的是：

```text
camera model: pinhole
distortion model: radtan
ROS distortion_model: plumb_bob
resolution: 640 × 400/目
```

因此不能把旧的 `left_equi.yaml/right_equi.yaml` 与当前 HB 外参混用。

## 3. 最新标定结果

### 3.1 标定质量

| 项目 | mean | median | std |
| --- | ---: | ---: | ---: |
| cam0 重投影误差 | 0.2783 px | 0.2559 px | 0.1633 px |
| cam1 重投影误差 | 0.2750 px | 0.2352 px | 0.1860 px |
| 陀螺仪残差 | 0.03020 rad/s | 0.02153 rad/s | 0.02999 rad/s |
| 加速度计残差 | 0.13610 m/s² | 0.10910 m/s² | 0.10817 m/s² |

两目重投影误差小于 `0.3 px`，相机部分标定可用。IMU 残差不算特别小，因此实际 VIO 仍需重点验证时间同步、IMU 轴方向、噪声参数、安装刚性和启动静止初始化，不能只根据重投影误差判断整套 VIO 一定稳定。

### 3.2 当前内参与双目基线

cam0：

```text
fx = 425.15104507345256
fy = 423.95324283344786
cx = 345.8498882720062
cy = 192.38144275949747
D  = [0.03348923360153752,
     -0.09139047589107362,
     -0.0028783135954485463,
      0.00005640401651295125]
```

cam1：

```text
fx = 418.5199287061255
fy = 417.1493453810506
cx = 330.3506254883004
cy = 195.80935672748853
D  = [0.025440046787188246,
     -0.06613770642141249,
     -0.0032043980489995776,
      0.001164758318933005]
```

双目基线：

```text
baseline = 0.05019105908868791 m ≈ 50.19 mm
```

整流后的公共投影参数：

```text
fx' = fy' = 474.56957021042615
cx' = 319.0171089172363
cy' = 191.44387245178223
P_right[0,3] = -23.819149340124724
```

可用下面的关系核对 CameraInfo 基线：

```text
baseline = -P_right[0,3] / P_right[0,0]
         = 23.8191493401 / 474.5695702104
         ≈ 0.0501911 m
```

### 3.3 当前相机—IMU 外参

HB 启动文件发布 `imu_link` 为父坐标系、`cam0/cam1` 为子坐标系的静态 TF。数值来自 Kalibr 的 `T_ic`，即将相机坐标中的点变换到 IMU 坐标系。

`imu_link → cam0`：

```text
translation (m):
x = -0.04834574
y = -0.02707217
z = -0.00892512

quaternion (x, y, z, w):
qx = -0.501654061932
qy = -0.471951072115
qz =  0.500099410078
qw =  0.524886623677
```

`imu_link → cam1`：

```text
translation (m):
x = -0.04748951
y =  0.02311057
z = -0.00924460

quaternion (x, y, z, w):
qx = -0.519104501097
qy = -0.493945949931
qz =  0.480755105280
qw =  0.505393355946
```

相机在 IMU 物理前方但平移 `x` 为负，不代表外参方向写反。该向量是在 IMU 自身坐标轴下表达的，而且相机与 IMU 之间有接近 90° 的轴向旋转，必须同时看完整旋转和平移。装到机器狗后还需要单独定义符合机器人前左上约定的 `base_link → imu_link`。

### 3.4 相机—IMU 时间偏移

Kalibr 结果：

```text
cam0: t_imu = t_cam + 0.04039484121924015 s
cam1: t_imu = t_cam + 0.04036685286347543 s
```

驱动使用两者平均值：

```text
camera_time_offset_ms = 40.38084704135779 ms
```

时间偏移方向是 `t_imu = t_cam + shift`。方向写反时，低速可能看不明显，大角速度运动时会迅速破坏视觉与 IMU 的一致性。

## 4. 今日完成的系统接入

新增 `hb_imu_rtabmap.launch.py`，一次启动以下节点：

```text
stereo_v4l2_direct_node
wit_imu_node
imu_to_hb_cam0 / imu_to_hb_cam1
cam0/rectify / cam1/rectify
openvins_stereo_odometry
rtabmap
rtabmap_viz
```

主要数据流：

```text
/cam0/image_raw + /cam0/camera_info(plumb_bob)
                 ↓ image_proc/rectify_node
             /cam0/image_rect

/cam1/image_raw + /cam1/camera_info(plumb_bob)
                 ↓ image_proc/rectify_node
             /cam1/image_rect

/cam0/image_rect + /cam1/image_rect + /imu/data_raw
                 ↓
     openvins_stereo_odometry
                 ↓
        /odom + /odom_info
                 ↓
              RTAB-Map
```

当前关键参数：

```text
Odom/Strategy                       = 10  # OpenVINS
OdomOpenVINS/UseStereo              = true
OdomOpenVINS/UseKLT                 = true
OdomOpenVINS/NumPts                 = 300
OdomOpenVINS/MinPxDist              = 10
OdomOpenVINS/MaxClones              = 15
OdomOpenVINS/MaxMSCKFInUpdate       = 80
OdomOpenVINS/MaxSLAMInUpdate        = 30
FAST/Threshold                      = 10
Rtabmap/ImagesAlreadyRectified      = true
always_process_most_recent_frame    = false
```

`always_process_most_recent_frame=false` 可以避免节点主动跳到最新帧而丢弃排队中的图像，但它不能解决画面模糊、弱纹理或滤波器已经发散的问题。

## 5. 今日代表性测试和判断过程

### 5.1 19:03:32：进入暗墙和走廊后持续发散

日志目录：

```text
/home/bird/.ros/log/2026-07-21-19-03-32-263201-bird-HP-Laptop-15-fd1xxx-49835
```

主要现象：

- 前约 39 秒基本正常。
- 约 43 秒，有效更新特征降到 `2`。
- 约 44 秒只剩 `1` 个特征，本地图点只剩 `4`。
- 约 48 秒变成 `0 特征 / 0 本地图点`。
- 之后长时间没有恢复，位置标准差持续增长。
- 高频累计距离错误增长到约 `116 m`。
- 没有接受有效全局回环，因此没有把错误轨迹拉回。

导出图像后看到：

```text
45 s 左右：昏暗玻璃门和墙面
48～50 s：近距离、低纹理暗墙占据大部分视野
54～60 s：暗走廊，主要结构只有重复的天花板灯带
```

传感器数据正常：相机约 `14.1 Hz`、IMU 约 `199.5 Hz`，没有持续丢帧，OpenVINS 平均计算时间约 `5 ms`，明显小于约 `70 ms` 的图像周期。结论是视觉几何约束持续失效，而不是 CPU、USB 或队列不足。

### 5.2 19:10:47：初始化后立即产生虚假速度

日志目录：

```text
/home/bird/.ros/log/2026-07-21-19-10-47-666954-bird-HP-Laptop-15-fd1xxx-58637
```

主要现象：

- 第一帧有效里程计输出时，估计速度已经约 `1.60 m/s`。
- 约 3 秒后估计位移约 `3.9 m`，但导出的图像几乎没有平移。
- 有效更新特征几乎一直是 `0`，只有两帧为 `1`。
- 最终输出位置约为 `(1.61, -10.51, -4.39) m`。
- 图优化轨迹约 `25 m`，高频累计距离约 `50 m`，都与实际运动不符。
- 平移标准差增长到约 `1.65 m`。

这次不是中途看向白墙后才丢失，而是静态初始化完成时状态已经不可信。当前 ZUPT 只在估计速度小于 `0.1 m/s` 时尝试；第一帧已经错误达到 `1.6 m/s`，因此后续 ZUPT 很难再触发。

这类问题应优先检查：

1. 启动前和初始化窗口内是否真正静止。
2. IMU 三轴方向、单位和重力模长是否正确。
3. 相机—IMU 时间偏移方向是否正确。
4. 外参是否与实际安装保持刚性。
5. 首个动作是否只有原地旋转而缺少平移视差。

### 5.3 19:21:19：前半段良好，靠近黑色显示器后丢失

日志目录：

```text
/home/bird/.ros/log/2026-07-21-19-21-19-477487-bird-HP-Laptop-15-fd1xxx-64575
```

主要统计：

```text
运行时长                 ≈ 57.6 s
RTAB-Map 节点            = 53
报告轨迹长度             ≈ 15.8 m
OpenVINS 正常段特征      ≈ 20～60
OpenVINS 正常段线性 std  ≈ 0.01～0.02 m
相机频率                 ≈ 14.30 Hz
IMU 频率                 ≈ 199.4～199.7 Hz
```

时间变化：

| 相对时间 | 有效更新特征 | LocalMap | 线性 std | 现象 |
| ---: | ---: | ---: | ---: | --- |
| 33.6 s | 34 | 57 | 0.013 m | 正常室内纹理 |
| 35.7 s | 10 | 32 | 0.014 m | 特征开始下降 |
| 36.7 s | 1 | 26 | 0.017 m | 接近危险状态 |
| 37.8 s | 0 | 86 | 0.036 m | 快速转向近距离黑色显示器 |
| 40.9 s | 0 | 76 | 0.065 m | 已经主要依靠 IMU 传播 |
| 44.0 s | 0 | 22 | 0.159 m | 速度和位置开始明显发散 |
| 47.2 s | 0 | 33 | 0.255 m | 错误速度曾达到约 2.58 m/s |

关键观察是：`Features=0` 时 `LocalMap` 仍可能有几十个点。这说明画面恢复后，前端确实重新检测或跟踪到了一部分点，但没有特征成功用于当前 MSCKF/SLAM 滤波更新。

数据库报告中的 `loops=13` 可能包含全局回环和邻近检测连接；同时 `Accepted_hypothesis_id` 一直为 0，因此不能只根据一个 `loops` 总数判断完成了可靠的全局重定位。

### 5.4 三次测试的共同结论

| 项目 | 判断 |
| --- | --- |
| 相机 USB 链路 | 基本正常，没有持续掉线或重连 |
| 图像发布频率 | 约 14.3 Hz，低于标称 15 Hz 但稳定 |
| IMU 发布频率 | 约 199.5 Hz，校验和、丢样和重同步基本为 0 |
| 算力 | OpenVINS 处理时间明显小于帧周期，不是主要瓶颈 |
| 正常纹理表现 | 可以达到 20～60 个有效更新特征和厘米级内部标准差 |
| 直接失效原因 | 低纹理、暗部、黑色显示器、纯旋转、大幅运动和过长曝光的组合 |
| 失效后不恢复原因 | OpenVINS 没有自动重定位和健康触发重置机制 |

## 6. `Features=0` 到底表示什么

当前 RTAB-Map OpenVINS 封装中的特征统计为：

```cpp
info->features = features_SLAM.size() + good_features_MSCKF.size();
```

这里统计的是成功进入滤波更新的 SLAM/MSCKF 特征，不是 FAST/KLT 在当前图像中检测到的所有角点。

新看到桌子和电脑后，角点需要经历以下过程：

```text
检测新角点
   ↓
跨多帧稳定跟踪
   ↓
满足视差、深度和三角化条件
   ↓
与当前预测状态计算重投影残差
   ↓
通过卡方检验
   ↓
进入 MSCKF/SLAM 更新并计入 Features
```

当位置、速度或 IMU 偏置已经发散时，新特征与错误预测状态不一致，容易在三角化或卡方检验阶段被拒绝。因此“重新看到丰富纹理”和“滤波器恢复正确状态”不是一回事。

经验参考：

| 有效更新特征 | 解释 |
| ---: | --- |
| `> 30` | 通常良好 |
| `15～30` | 可用，应结合速度和标准差判断 |
| `5～15` | 危险，持续出现容易漂移 |
| `< 5` 持续约 0.5～1 s | 应进入健康告警或重置流程 |
| `0` | 当前没有视觉滤波更新，主要依靠 IMU 传播 |

不要把日志中的 `quality=0` 单独当成跟踪失败。当前 OpenVINS 封装中该字段并不能完整代表前端检测质量，应结合 `Odometry/Features`、`Odometry/LocalMapSize`、协方差、速度和原始图像一起判断。

## 7. 为什么重新看到旧场景不会自动恢复

### 7.1 KLT 不是重定位器

KLT 只负责短时间的相邻帧跟踪。旧特征丢失后，新检测角点会得到新的 ID，它不知道当前电脑、桌子是以前见过的对象。OpenVINS 本身没有词袋、旧地图搜索或全局 PnP 重定位模块。

### 7.2 OpenVINS 初始化后不会因特征为 0 自动重新初始化

OpenVINS 的 `VioManager` 只在 `is_initialized_vio=false` 时执行初始化。正常运行后，即使有效更新特征连续为 0，它仍保持已初始化状态并继续用 IMU 传播。

RTAB-Map 的 `OdometryOpenVINS` 封装只要能从 OpenVINS 状态中得到非空姿态，就继续返回非空变换。即使 `info->features=0`，ROS 里程计节点也不会判定 `lost=true`。

因此当前：

```text
Odom/ResetCountdown = 10
```

也不会自动解决问题，因为 `ResetCountdown` 只在里程计连续返回空变换时触发，而 OpenVINS 仍在输出 IMU 推算姿态。

### 7.3 ZUPT 不能修复已经发散的全局位置

当前设置：

```text
TryZUPT             = true
ZUPTMaxVelodicy     = 0.1 m/s
ZUPTMaxDisparity    = 0.5 px
ZUPTOnlyAtBeginning = false
```

ZUPT 只能在系统判断设备静止时约束速度。如果系统已经错误估计到 `1～2 m/s`，或者画面仍有超过 `0.5 px` 的运动视差，就不会触发。即使后来成功把速度压回零，也无法知道此前错误积分的位置应该回到哪里。

### 7.4 RTAB-Map 回环修正的是地图层

RTAB-Map 回环通过修改：

```text
map → odom
```

修正机器人在全局地图中的位置，但不会修改 OpenVINS 内部的速度、IMU 偏置和滑动窗口状态。如果 `odom → imu_link` 继续高速发散，单靠 `map → odom` 抵消不是稳定的长期方案。

## 8. 可实施的失效恢复和重定位方案

推荐增加一个视觉健康状态机：

```text
NORMAL
  │
  ├─ Features < 5 持续 10～15 帧
  ├─ 且 StdDevLin > 0.05～0.10 m 或持续增长
  └─ 或出现不合理速度
          ↓
        LOST
          ↓
保存最后可信位姿、停止采用错误增量
          ↓
重置 OpenVINS 并重新初始化
          ↓
建立新的局部 odom 段
          ↓
RTAB-Map 通过旧场景把新段对齐到旧地图
```

当前节点已经提供：

```text
/openvins_stereo_odometry/reset_odom
/openvins_stereo_odometry/reset_odom_to_pose
```

不建议失效后盲目调用空的 `reset_odom`，因为它可能造成里程计坐标跳回原点。更合理的实现是在节点内部保存最后可信位姿，以该位姿重建 OpenVINS，并向 RTAB-Map 发布足够大的协方差或开始新子图，避免把失效期间的错误边继续写入旧图。

机器狗部署时建议在 VIO 进入 `LOST` 后短暂停止运动，等待约 `2～3 s` 静态初始化。若必须边走边恢复，需要启用和重新验证动态初始化，风险明显更高。机器狗还可以用腿式里程计作为短时外部约束，这通常比仅靠消费级 IMU 长时间积分更有效。

已有地图定位模式需要保证：

```text
delete_db_on_start       = false
Mem/IncrementalMemory    = false
Mem/LocalizationReadOnly = true
Mem/InitWMWithAllNodes   = true
```

继续扩展旧地图时保持 `Mem/IncrementalMemory=true`，不要误切成只读定位模式。

## 9. 是否切换 RTAB-Map VINS-Fusion

当前 RTAB-Map 源码支持：

```text
Odom/Strategy=9   VINS-Fusion
Odom/Strategy=10  OpenVINS
```

但当前构建缓存为：

```text
WITH_VINS_FUSION:BOOL=OFF
```

因此目前不能只把参数改成 `9`，需要先配置依赖并重新编译 RTAB-Map 和相关 `rtabmap_ros` 包。

### 9.1 与当前问题相关的差异

| 能力 | OpenVINS | RTAB-Map 内置 VINS-Fusion |
| --- | --- | --- |
| 核心方法 | EKF/MSCKF | 滑动窗口非线性优化 |
| 正常阶段 | 延迟低、协方差可用 | 可能更平滑，但计算量更高 |
| 短暂少特征 | IMU 传播 | 滑窗可能保留更多历史约束 |
| 旧地图重定位 | 不支持 | RTAB-Map 封装同样不支持 |
| 内置回环 | 无 | 封装只接 estimator，没有接 `loop_fusion` |
| 自动失败重启 | 当前无 | 本机源码中也被关闭 |
| 鱼眼输入 | 可支持 equidistant 或校正图 | RTAB-Map 封装要求校正后的 pinhole 图 |

本机 VINS-Fusion 源码中的 `failureDetection()` 一开始直接 `return false`，后续特征过少和 IMU 偏置过大的检查不会运行。另外，RTAB-Map VINS-Fusion 封装在初始化后将协方差固定为 `0.0001`，不适合直接用来判断是否发散。

因此 VINS-Fusion 值得作为同路线 A/B 实验，但不能把它当作自动重定位方案。即使切换，仍建议实现健康检测、主动重置和 RTAB-Map 地图级重定位。

## 10. 相机质量和曝光判断

### 10.1 为什么不能直接断定相机硬件不行

19:21 测试的前半段能够保持：

```text
有效更新特征 = 20～60
线性 std      = 0.01～0.02 m
```

左右目校正图同步、结构一致，相机发布稳定，说明相机、标定和驱动在一般场景下可以工作。当前问题更像是图像质量上限、参数和场景共同作用，而不是相机完全损坏。

### 10.2 当前相机的客观限制

```text
每目分辨率约 640 × 400
实际帧率约 14.3 Hz
室内画面偏暗
近距离黑色物体和无纹理墙面容易占据大部分视野
```

与高帧率、短曝光、宽动态范围更好的相机相比，这套相机在快速甩动时的视觉余量更小。但在修正曝光、增益和照明前，不应先下结论更换硬件。

### 10.3 今日发现的相机控制问题

19:21 运行日志实际记录：

```text
auto_exposure             = 1  # 手动曝光
exposure_time_absolute    = 10000
gain                      = 1
sharpness                 = 0
contrast                  = 0
```

V4L2 绝对曝光单位通常为 `100 μs`。按该单位，`150` 对应 `15 ms`，而 `10000` 对应名义上的 `1 s`，远大于 15 Hz 的 `66.7 ms` 帧周期。设备可能在内部限幅，但这种设置仍不适合 VIO。

写本文时相机启动源码已变为：

```text
exposure_time_absolute = 5000
gain                   = 128
```

曝光仍与旁边“`150` 对应 `15 ms`”的注释不一致，应继续修正并通过运行日志确认最终生效值。

建议调参顺序：

1. 优先增加环境照明。
2. 将曝光从 `150～200` 起步，即约 `15～20 ms`。
3. 根据亮度逐步增加增益，避免一开始就拉到最大。
4. 做相同路线和相同甩动速度的 A/B 测试。
5. 同时比较有效特征、线性标准差和导出图像，不能只凭肉眼看 RViz。

短曝光会使画面更暗，高增益会增加噪声，因此最有效的手段通常是补光，而不是在曝光和增益之间无限折中。

## 11. 最简单的日志定位方法

### 11.1 记住三个位置

```text
launch 启停日志：~/.ros/log/日期时间-主机-PID/launch.log
节点详细日志：  ~/.ros/log/节点名_PID_时间戳.log
建图运行结果：  ~/.ros/*.db
```

ROS 2 运行结束后，先找最新 launch：

```bash
L=$(find ~/.ros/log -mindepth 1 -maxdepth 1 -type d \
  -name '20*' -printf '%T@ %p\n' | sort -nr | sed -n '1p' | cut -d' ' -f2-)
printf '%s\n' "$L"
sed -n '1,100p' "$L/launch.log"
```

如果第三条命令没有输出，先检查：

```bash
ls -la "$L"
```

有些启动目录只创建了目录但尚未写入 `launch.log`，也可能是取到了仍在运行或异常启动的最新目录。此时向前查看几个目录：

```bash
find ~/.ros/log -mindepth 1 -maxdepth 1 -type d \
  -name '20*' -printf '%T@ %p\n' | sort -nr | sed -n '1,5p'
```

### 11.2 一次配置一个简单函数

可以把下面函数放进 `~/.bashrc`。以后只需要记住 `ros_latest_logs`。

```bash
# 找到最新 ROS 2 launch，并列出本项目四个关键节点的详细日志。
ros_latest_logs()
{
  # 最新目录按修改时间选择，避免依赖目录名字排序。
  local launch_dir
  launch_dir=$(find ~/.ros/log -mindepth 1 -maxdepth 1 -type d \
    -name '20*' -printf '%T@ %p\n' | sort -nr | sed -n '1p' | cut -d' ' -f2-)

  printf 'launch: %s\n' "$launch_dir"
  sed -n '1,100p' "$launch_dir/launch.log" 2>/dev/null

  # 根据 launch.log 中的 PID 精确找到同一次运行的节点日志。
  local node pid
  for node in stereo_v4l2_direct_node wit_imu_node stereo_odometry rtabmap
  do
    pid=$(sed -nE \
      "s/.*\\[${node}-[0-9]+\\]: process started with pid \\[([0-9]+)\\].*/\\1/p" \
      "$launch_dir/launch.log" | sed -n '1p')
    find ~/.ros/log -maxdepth 1 -type f \
      -name "${node}_${pid}_*.log" -print 2>/dev/null
  done
}
```

让函数立即生效：

```bash
source ~/.bashrc
ros_latest_logs
```

## 12. 固定的排查顺序

以后不要一看到漂移就先改回环阈值。按下面顺序判断：

```text
1. launch 是否正常退出
       ↓
2. 相机频率、丢帧、曝光和实际画面
       ↓
3. IMU 频率、丢样、校验和和时间相位
       ↓
4. OpenVINS 特征、LocalMap、标准差和速度
       ↓
5. RTAB-Map 候选回环、几何验证和图优化
       ↓
6. 导出图像和轨迹复核触发场景
```

### 12.1 检查 launch

正常停止应看到：

```text
user interrupted with ctrl-c (SIGINT)
process has finished cleanly
```

若有 `exited with code`、`process has died` 或节点未启动，先解决启动问题，再分析算法效果。

### 12.2 检查相机

```bash
camera_log=$(ls -t ~/.ros/log/stereo_v4l2_direct_node_*.log | sed -n '1p')
rg -n -i \
  'configured|camera control|direct capture|fps|drop|stall|timeout|reconnect|warn|error' \
  "$camera_log"
```

重点看：

- 实际 FPS 是否稳定。
- `kernel sequence drops` 是否持续增长。
- 是否出现 `stall/timeout/reconnect`。
- 本次实际曝光、增益和锐度是多少。
- 左右目是否同亮度、同步和清晰。

频率正常不代表图像适合 VIO。必须继续导出图像确认运动模糊、暗部、反光、白墙和动态人物。

### 12.3 检查 IMU

```bash
imu_log=$(ls -t ~/.ros/log/wit_imu_node_*.log | sed -n '1p')
rg -n -i \
  'imu:|hz|checksum|discarded|incomplete|missing|resync|phase|warn|error' \
  "$imu_log"
```

正常参考：

```text
频率                  ≈ 200 Hz
checksum errors       = 0
missing samples       = 0
timestamp resyncs     = 0
phase error           稳定在数毫秒内
```

启动时少量 `discarded bytes` 不一定造成整段漂移；如果错误持续增长或时间戳重同步，才应优先处理 IMU 驱动。

### 12.4 检查 OpenVINS 标准差

```bash
odom_log=$(ls -t ~/.ros/log/stereo_odometry_*.log | sed -n '1p')
rg -n 'std dev=' "$odom_log" | tail -n 80
```

经验参考：

| 线性 std | 判断 |
| ---: | --- |
| `< 0.02 m` | 通常良好 |
| `0.02～0.05 m` | 注意，结合特征数和运动判断 |
| 持续 `> 0.05 m` | 视觉约束明显变弱 |
| `> 0.10 m` 且持续增长 | 很可能已经发散 |

启动前的 `0/0` 和初始化附近的 `1/1` 可能是占位值，统计时要排除。

快速汇总：

```bash
sed -nE 's/.*std dev=([0-9.]+)m\|([0-9.]+)rad.*/\1 \2/p' "$odom_log" |
awk '
  $1 > 0 && $1 < 0.99 {
    n++
    if ($1 > max_l) max_l=$1
    if ($2 > max_a) max_a=$2
    if ($1 > 0.05) bad_l++
    if ($2 > 0.20) bad_a++
  }
  END {
    printf("samples=%d max_linear=%.6f max_angular=%.6f linear_gt_5cm=%d angular_gt_0.2rad=%d\n",
      n, max_l, max_a, bad_l, bad_a)
  }'
```

### 12.5 运行时观察特征和 TF

```bash
ros2 topic hz /cam0/image_rect
ros2 topic hz /cam1/image_rect
ros2 topic hz /imu/data_raw
ros2 topic echo /odom_info --field features
ros2 run tf2_ros tf2_echo map odom
```

也可直接画曲线：

```bash
rqt_plot /odom_info/features
```

如果需要区分“前端仍有活跃轨迹”和“成功进入滤波更新的特征”，还应同时查看数据库里的：

```text
Odometry/Features/
Odometry/LocalMapSize/
Odometry/StdDevLin/
Odometry/Speed/mps
```

### 12.6 从数据库看整次运行

先加载当前编译版本：

```bash
source /opt/ros/humble/setup.bash
source ~/rtabmap_humble_ws/install/setup.bash
db=~/.ros/rtabmap_openvins_hb_mapping.db
```

最快的总体报告：

```bash
report_dir=$(mktemp -d /tmp/rtabmap_report.XXXXXX)
(
  cd "$report_dir" || exit 1
  QT_QPA_PLATFORM=offscreen rtabmap-report --report "$db"
)
printf '报告文件：%s/report.txt\n' "$report_dir"
```

`--report` 会把结果写成 `report.txt`，因此这里先进入独立临时目录，避免忘记文件生成在什么位置。

查看数据库包含哪些统计项：

```bash
QT_QPA_PLATFORM=offscreen rtabmap-report --stats "$db" |
rg 'Odometry/|Loop/|Memory/Distance|MapToBase'
```

导出某一个统计量：

```bash
stat_dir=$(mktemp -d /tmp/rtabmap_stats.XXXXXX)
(
  cd "$stat_dir" || exit 1
  QT_QPA_PLATFORM=offscreen rtabmap-report \
    'Odometry/Features/' --export --export_prefix features "$db"
)
printf '%s\n' "$stat_dir"
```

最常用的统计项：

```text
Odometry/Features/
Odometry/LocalMapSize/
Odometry/StdDevLin/
Odometry/Speed/mps
Memory/Distance_travelled/m
Loop/Visual_matches/
Loop/Accepted_hypothesis_id/
Loop/RejectedHypothesis/
```

不要只看 `loops=` 总数。要同时判断是否有接受的假设、视觉内点是否足够、图优化是否接受约束，以及 `map → odom` 是否实际发生变化。

### 12.7 导出左右目图像

```bash
image_dir=$(mktemp -d /tmp/rtabmap_images.XXXXXX)
QT_QPA_PLATFORM=offscreen rtabmap-export \
  --images_id \
  --output hb_run \
  --output_dir "$image_dir" \
  "$db"
printf '%s\n' "$image_dir"
```

重点查看漂移前后约 5 秒：

- 是否发生大幅转动或纯旋转。
- 是否有近距离黑色显示器、白墙或地板占满视野。
- 是否运动模糊或曝光拉丝。
- 左右目亮度和内容是否一致。
- 是否主要看到玻璃、反光和重复灯带。
- 是否有动态人物占据主要特征区域。

图像是判断“相机质量问题还是场景问题”最直接的证据。

### 12.8 导出优化前后轨迹

```bash
QT_QPA_PLATFORM=offscreen rtabmap-report --poses_raw "$db"
```

通常生成：

```text
*_odom.txt   OpenVINS 原始里程计轨迹
*_slam.txt   RTAB-Map 图优化后轨迹
```

如果两条轨迹都漂，说明 RTAB-Map 没有获得足够的全局约束；如果 `slam` 明显回到正确位置而 `odom` 仍漂，说明 RTAB-Map 回环有效，但局部 VIO 没有恢复。

## 13. 快速判断表

| 现象 | 更可能的原因 | 下一步 |
| --- | --- | --- |
| 相机 FPS 明显下降、drop 持续增长 | USB、驱动或曝光改变帧周期 | 先解决相机链路 |
| IMU missing/resync 持续增长 | 串口、时间戳或驱动问题 | 先解决 IMU 链路 |
| 画面清晰、Features 高、std 小但地图弯 | 外参、时间偏移、图优化或尺度问题 | 检查标定和 TF |
| Features 瞬间降 0，几帧后恢复且 std 不增长 | 短暂遮挡或模糊 | 先观察，不急着重置 |
| Features 持续为 0、std 和速度增长 | VIO 已进入纯 IMU 传播 | 停止采用位姿并重置 VIO |
| LocalMap 有点但 Features 为 0 | 检测/跟踪存在，但更新被拒绝 | 看三角化、残差、状态是否已发散 |
| 有 Visual matches 但 Accepted hypothesis 为 0 | 外观候选未通过几何验证 | 看内点和图一致性，不要盲目降阈值 |
| `map → odom` 修正但 `odom → base` 仍乱跑 | RTAB-Map 在修地图，VIO 内部仍发散 | 重置局部 VIO |
| 静止初始化后立即出现 1 m/s 以上速度 | 初始化、IMU 轴/单位、时间偏移或外参问题 | 固定设备重做静止启动测试 |

## 14. 参数优化优先级

### 14.1 第一优先级：图像质量

```text
曝光起点              = 150～200（约 15～20 ms）
帧率目标              = 稳定 15 Hz，能提高则优先 20 Hz 以上
增益                  = 在亮度足够前提下尽量低
照明                  = 优先补光
自动曝光              = VIO 对比测试中保持关闭
```

每次只改一组参数，记录运行日志中的实际控制值。不要只修改 launch 源码后就认为已经生效。

### 14.2 第二优先级：运动方式和初始化

1. 启动前将设备刚性固定并完全静止。
2. 启动后继续静止 `8～10 s`。
3. 第一个动作先缓慢前后平移约 `0.5 m`，不要先原地快速旋转。
4. 正常测试时控制相同路线和速度，便于 A/B 对比。
5. 专门设计一次黑色显示器、白墙和走廊压力测试，但不要与正常基准测试混在一起。

### 14.3 第三优先级：OpenVINS 健康检测

建议初始阈值：

```text
features_warn       < 10
features_lost       < 5，持续 10～15 帧
std_linear_lost     > 0.08 m，或连续增长
speed_implausible   根据实际机器人最大速度设置
```

不要仅凭单帧 `Features=0` 重置，否则快速遮挡或偶发模糊会造成频繁重启。应至少组合特征持续时间、协方差和速度两个以上条件。

### 14.4 RTAB-Map 回环参数

当前回环参数已经能够产生候选和邻近连接。VIO 已经发散时，直接降低 `Vis/MinInliers`、降低 `Rtabmap/LoopThr` 或放宽图优化误差阈值，可能把错误约束写入地图。

调回环前应先确认：

1. 当前图像确实与旧节点相似。
2. Visual matches 和 inliers 足够。
3. 候选通过几何验证。
4. 候选与当前图的误差不是由前端数十米漂移造成的。

### 14.5 点云降噪参数

当前低噪点配置：

```text
Stereo/DenseStrategy               = 0  # StereoBM
StereoBM/BlockSize                 = 15
StereoBM/NumDisparities            = 96
StereoBM/UniquenessRatio           = 20
StereoBM/SpeckleWindowSize         = 200
Grid/RangeMin                      = 0.3 m
Grid/RangeMax                      = 3.0 m
Grid/CellSize                      = 0.05 m
Grid/NoiseFilteringRadius          = 0.1 m
Grid/NoiseFilteringMinNeighbors    = 5
```

约 `50 mm` 基线在远距离的深度误差增长很快。如果仍有大量飞点，优先把 `Grid/RangeMax` 降至 `2.5 m`，再考虑增强邻域过滤，不要先追求更密的深度。

### 14.6 机器狗上的 `Reg/Force3DoF`

当前保持：

```text
Reg/Force3DoF = false
```

四足机器人会有明显俯仰和横滚，建图和 VIO 阶段通常不应强制 3DoF。只有下游明确只需要平面导航，并且已经有稳定的重力对齐和机体姿态处理时，才考虑在合适层级约束到平面，而不是简单截掉原始 VIO 的 z、roll 和 pitch。

## 15. 建议的标准 A/B 测试流程

1. 给每次实验使用独立数据库文件，避免覆盖上一轮证据。
2. 记录启动命令、曝光、增益、帧率、标定版本和参数文件 Git 状态。
3. 固定设备静止 `8～10 s` 完成初始化。
4. 缓慢平移后再开始正常转弯。
5. 依次经过纹理丰富区、弱纹理区、走廊和回到起点。
6. 结束后先保存 launch 目录和四个节点日志路径。
7. 运行数据库总体报告。
8. 导出 `Features/LocalMap/StdDev/Speed`。
9. 导出漂移前后图像。
10. 导出 `odom/slam` 轨迹对比。
11. 一次只修改一组参数，再走完全相同路线。

建议数据库命名：

```bash
db="$HOME/.ros/hb_test_$(date +%F_%H%M%S).db"
ros2 launch stereo_camera_pkg_py hb_imu_rtabmap.launch.py \
  database_path:="$db" \
  delete_db_on_start:=true
```

继续旧地图时必须使用同一路径并设置：

```text
delete_db_on_start = false
```

## 16. 后续建议

按优先级执行：

1. 将 HB 相机曝光真正降到 `150～200`，合理设置增益并增加补光。
2. 让 `hb_imu_rtabmap.launch.py` 明确声明并转发曝光、增益等相机参数，避免修改子 launch 后难以确认最终值。
3. 用固定设备、静止启动和缓慢平移完成一组基准测试。
4. 实现 OpenVINS 健康检测和保持最后可信位姿的重置逻辑。
5. 让 RTAB-Map 在 OpenVINS 重置后创建新子图，并验证重新看到旧场景时能否合并。
6. 如果仍需比较，再启用 `WITH_VINS_FUSION=ON` 建立独立 VINS-Fusion A/B 版本；不要直接替换当前可运行配置。
7. 上机器狗后接入腿式里程计或其他速度约束，避免视觉失效期间仅靠 IMU 长时间积分。

## 17. 一页式检查清单

运行前：

- [ ] 确认当前使用 HB 的 `left_hb.yaml/right_hb.yaml`。
- [ ] 确认 `camera_time_offset_ms=40.38084704135779`。
- [ ] 确认 `imu_link → cam0/cam1` 是最新外参。
- [ ] 确认曝光和增益不是意外的 `10000/1` 或其他极端值。
- [ ] 确认数据库是否需要删除。
- [ ] 设备固定且初始化阶段静止。

运行中：

- [ ] 相机约 14.3～15 Hz 且没有持续 drop。
- [ ] IMU 约 200 Hz 且没有 missing/resync。
- [ ] 有效更新特征尽量保持在 15 以上。
- [ ] 线性 std 尽量保持在 0.05 m 以下。
- [ ] 避免黑色显示器、白墙或地板长时间占满画面。
- [ ] 避免先纯旋转再平移。

运行后：

- [ ] 保存最新 launch 路径和节点 PID。
- [ ] 检查相机、IMU 和 odom 日志。
- [ ] 用 `rtabmap-report --report` 看总体结果。
- [ ] 导出 Features、LocalMap、StdDev 和 Speed。
- [ ] 导出漂移前后左右目图像。
- [ ] 对比原始 odom 与优化后 slam 轨迹。
- [ ] 记录本次只修改了哪些参数。
