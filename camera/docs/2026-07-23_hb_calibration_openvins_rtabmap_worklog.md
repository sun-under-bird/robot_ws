# 2026-07-23 HB 双目标定、OpenVINS 与 RTAB-Map 工作总结

本文记录 2026 年 7 月 23 日围绕 HB USB 双目、WIT IMU、OpenVINS 和 RTAB-Map 完成的工作，同时总结本次排查中形成的日志分析、源码定位和标定接入经验。

## 1. 当日工作概览

当天完成了以下五项主要工作：

1. 修正 OpenVINS 使用矫正图像时，相机外参仍属于原始相机坐标系的问题。
2. 分析圆周运动容易漂移、左右平移相对稳定以及里程计可视化不显示跟踪点的原因。
3. 解释 `delete_db_on_start=false` 时旧地图逐步恢复，而不是一次性完整显示的原因。
4. 重新完成一套每目 `640×480` 双目标定和相机—IMU 联合标定，并生成 ROS `CameraInfo`。
5. 新建使用本次标定的独立启动文件，并通过多轮运行检查相机、IMU、OpenVINS 和 RTAB-Map 的工作状态。

当天最重要的改进，是把 OpenVINS 和 RTAB-Map 的图像输入明确拆分：

```text
OpenVINS：
  原始图像 + 原始 CameraInfo K/D + Kalibr 原始相机外参

RTAB-Map：
  极线矫正图像 + CameraInfo P/R + OpenVINS 输出的 /odom
```

这样可以避免把原始相机外参与经过 `image_proc` 旋转后的矫正图像坐标轴混在一起。

## 2. 主要文件索引

| 用途 | 文件 |
| --- | --- |
| 2026-07-23 新标定启动文件 | `/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/launch/hb_imu_rtabmap_20260723.launch.py` |
| OpenVINS 与 RTAB-Map 通用启动文件 | `/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/launch/rtabmap_openvins_stereo_mapping.launch.py` |
| OpenVINS 与 RTAB-Map 参数 | `/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/config/rtabmap_openvins_mapping_params.yaml` |
| 左目 640×480 CameraInfo | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/config/left_hb_480.yaml` |
| 右目 640×480 CameraInfo | `/home/bird/robot_ws/imu_ws/src/stereo_v4l2_camera/config/right_hb_480.yaml` |
| 双目标定结果 | `/home/bird/kalibr_data/stereo_calib-results-cam.txt` |
| 双目标定 camchain | `/home/bird/kalibr_data/stereo_calib-camchain.yaml` |
| 相机—IMU 联合标定结果 | `/home/bird/kalibr_data/cam_imu_repeat_01-results-imucam.txt` |
| 相机—IMU 联合标定 camchain | `/home/bird/kalibr_data/cam_imu_repeat_01-camchain-imucam.yaml` |
| 双目标定报告 | `/home/bird/kalibr_data/stereo_calib-report-cam.pdf` |
| 联合标定报告 | `/home/bird/kalibr_data/cam_imu_repeat_01-report-imucam.pdf` |
| RTAB-Map 数据库 | `/home/bird/.ros/rtabmap_openvins_hb_mapping.db` |

## 3. 原始图像、矫正图像和相机外参不能混用

### 3.1 问题来源

Kalibr 输出的相机—IMU 外参属于原始相机光学坐标系。`image_proc` 对双目图像做极线矫正时，会使用 `CameraInfo.rectification_matrix` 对图像射线和相机坐标轴进行旋转。

本次左目矫正矩阵为：

```text
[ 0.9991543,  0.0007367,  0.0411115]
[ 0.0008841,  0.9992234, -0.0393929]
[-0.0411086,  0.0393960,  0.9983777]
```

对应的坐标轴旋转约为 `3.3°`。右目的旋转也约为 `3°`。

如果 OpenVINS 同时使用：

```text
矫正后的图像
+ 原始相机 CameraInfo K/D
+ Kalibr 原始相机外参
```

那么视觉观测射线和外参所描述的相机坐标系不一致。误差只有几度时，静止或简单平移可能不明显，但连续转弯、画圆以及快速旋转时容易被放大。

### 3.2 完成的修正

在通用启动文件中增加了 OpenVINS 独立输入参数：

```text
odom_left_image_topic
odom_right_image_topic
odom_left_info_topic
odom_right_info_topic
odom_images_already_rectified
```

HB 启动文件现在使用：

```text
OpenVINS 左图：/cam0/image_raw
OpenVINS 右图：/cam1/image_raw
OpenVINS CameraInfo：/cam0/camera_info、/cam1/camera_info
Rtabmap/ImagesAlreadyRectified：false

RTAB-Map 左图：/cam0/image_rect
RTAB-Map 右图：/cam1/image_rect
```

OpenVINS 因而直接使用原始 `radtan` 图像、`K/D` 和原始相机外参；RTAB-Map 仍然使用极线矫正图像计算双目深度和点云。

### 3.3 从这次问题得到的经验

一套视觉标定参数必须保持以下四者属于同一坐标系：

```text
图像像素
内参
畸变模型
相机外参
```

只修改图像是否矫正，而不相应旋转外参，是一种很隐蔽的问题。检查 `CameraInfo` 时不能只看 `K`、`D` 和 `P`，还必须看 `R` 是否为单位矩阵。

## 4. 圆周运动比左右运动更容易漂移

左右平移时，已有特征通常能在视野中保持较长时间，而且水平方向视差明显，双目深度和运动约束较强。

画圆或连续转弯同时包含：

- 持续角速度；
- 相机视野不断替换；
- 特征跨帧寿命缩短；
- 走廊、墙面等低纹理区域；
- 人物遮挡和动态目标；
- 相机—IMU时间偏移及外参旋转误差被持续激发。

因此，圆周运动对下列问题更加敏感：

```text
外参坐标系是否一致
时间同步是否准确
陀螺仪噪声参数是否合理
曝光时间是否导致运动模糊
KLT 特征是否能跨帧持续跟踪
```

这也是为什么简单左右运动正常，不能证明相机—IMU标定和时间同步完全正确。标定后的验证动作应包含不同方向平移、正反向转弯、俯仰、回到起点和短暂静止。

## 5. 为什么 Odometry 中看不到 OpenVINS 跟踪点

改为原始图像并设置：

```text
Rtabmap/ImagesAlreadyRectified=false
```

之后，`rtabmap_viz` 的 Odometry 面板不再显示原来的跟踪点。

从 `OdometryOpenVINS.cpp` 源码确认：

- OpenVINS 仍然会统计 `features_SLAM` 和 `good_features_MSCKF`；
- `info.features` 和 `info.localMapSize` 仍会正常填写；
- 只有 `imagesAlreadyRectified()==true` 时，包装层才会生成用于可视化的 `newCorners` 和 `refCorners`。

因此，“看不见点”是当前包装层没有给原始图像模式生成可视化角点，不等于 OpenVINS 没有跟踪特征。

同样需要注意：

```text
Odom: quality=0
```

也不等于零特征。OpenVINS 包装层没有给 `info.reg.inliers` 赋值，而 ROS 日志中的 `quality` 正是打印该字段，所以它会一直显示为 0。判断 OpenVINS 前端状态应看数据库中的：

```text
Odometry/Features
Odometry/LocalMapSize
```

不能只看 `quality=0`。

## 6. `delete_db_on_start=false` 时旧地图为什么逐步恢复

`delete_db_on_start=false` 只表示启动时保留并打开原数据库，并不代表把数据库中的全部节点、图像和点云立即载入工作内存。

当前参数包括：

```text
Mem/IncrementalMemory=true
Mem/InitWMWithAllNodes=false
Rtabmap/TimeThr=700
```

其中：

- `Mem/IncrementalMemory=true` 表示继续进行增量建图；
- `Mem/InitWMWithAllNodes=false` 表示启动时不把全部历史节点装入 Working Memory；
- `Rtabmap/TimeThr=700` 会限制单次 RTAB-Map 更新的处理时间；
- 三维点云和占据栅格还需要根据当前载入的节点重新组织和发布。

因此，重新进入旧区域或发生定位匹配后，相关历史节点会从数据库/LTM逐步取回，视觉上就像旧地图一点一点出现。

如果目标是继续建图，这种按需载入方式可以降低启动时间和内存占用。如果只是查看完整旧地图，优先使用数据库查看工具；如果要做纯定位，再单独测试定位模式，不要仅为了“立即显示整幅地图”就直接把所有节点强制装入工作内存。

## 7. 2026-07-23 新双目标定结果

### 7.1 相机模型和内参

本次每目图像分辨率为：

```text
640×480
```

相机模型为：

```text
pinhole-radtan
```

左目：

```text
fx = 529.340491
fy = 528.423158
cx = 357.305431
cy = 229.281218

D = [0.05394312, -0.14364541, -0.00402823, 0.00224726]
```

右目：

```text
fx = 522.677922
fy = 521.388484
cx = 336.638237
cy = 234.249178

D = [0.03501952, -0.08697379, -0.00393002, 0.00375479]
```

双目基线：

```text
0.05028488 m
```

即约 `50.285 mm`，与相机实际约 50 mm 的基线相符。

### 7.2 联合标定重投影和 IMU 残差

| 项目 | 结果 |
| --- | ---: |
| cam0 平均重投影误差 | `0.41509 px` |
| cam1 平均重投影误差 | `0.41452 px` |
| 陀螺仪原始残差均值 | `0.02754 rad/s` |
| 加速度计原始残差均值 | `0.14314 m/s²` |
| 陀螺仪归一化残差均值 | `19.47` |
| 加速度计归一化残差均值 | `1.27` |

相机重投影误差和双目基线合理，说明相机几何部分总体可用。

需要特别注意，陀螺仪归一化残差 `19.47` 明显过大。这更像是联合标定使用的陀螺仪噪声密度 `0.0001` 过于乐观，而不是单凭这个数字就能断定相机外参错误。加速度计归一化残差接近 1，相对合理。

### 7.3 相机—IMU 时间偏移

Kalibr 定义：

```text
t_imu = t_cam + shift
```

结果为：

```text
cam0 shift = 40.97637054 ms
cam1 shift = 40.91608669 ms
平均值       = 40.94622861 ms
```

启动文件使用两目平均值：

```text
camera_time_offset_ms = 40.9462286143
```

### 7.4 启动文件采用的静态外参

`imu_link -> cam0`：

```text
translation:
  x = -0.045834755216
  y = -0.028481542246
  z = -0.014178159943

quaternion:
  x = -0.503285699174
  y = -0.466494277200
  z =  0.498289769958
  w =  0.529899895737
```

`imu_link -> cam1`：

```text
translation:
  x = -0.044470284056
  y =  0.021784055571
  z = -0.014455302437

quaternion:
  x = -0.520090859642
  y = -0.489617229383
  z =  0.478902038541
  w =  0.510326663902
```

这里的平移分量是带坐标轴方向的向量，不能只根据“相机在 IMU 前方”就要求某一个数必须为正。必须先确认变换方向以及 IMU、相机光学坐标系各轴的定义。

## 8. 新建启动文件

新增文件：

```text
/home/bird/robot_ws/camera/src/stereo_camera_pkg_py/launch/
hb_imu_rtabmap_20260723.launch.py
```

该启动文件完成：

- 启动 HB USB 双目相机；
- 使用本次 `left_hb_480.yaml` 和 `right_hb_480.yaml`；
- 给相机时间戳补偿 `40.9462286143 ms`；
- 启动 WIT IMU；
- 发布本次 `imu_link -> cam0/cam1` 静态 TF；
- 启动左右目 `image_proc/rectify_node`；
- 传感器先运行 5 秒，再启动 OpenVINS 和 RTAB-Map；
- OpenVINS 使用原始图像，RTAB-Map 使用矫正图像；
- 使用独立数据库 `~/.ros/rtabmap_openvins_hb_mapping.db`。

运行命令：

```bash
source /home/bird/robot_ws/install/setup.bash
ros2 launch stereo_camera_pkg_py hb_imu_rtabmap_20260723.launch.py
```

从空数据库测试：

```bash
ros2 launch stereo_camera_pkg_py hb_imu_rtabmap_20260723.launch.py \
  delete_db_on_start:=true
```

继续使用旧数据库：

```bash
ros2 launch stereo_camera_pkg_py hb_imu_rtabmap_20260723.launch.py \
  delete_db_on_start:=false
```

## 9. 当日运行结果

### 9.1 17:01 长距离测试

该次运行确认：

```text
相机：约 14.30～14.31 FPS
相机内核序号丢帧：0
IMU：约 199.4～199.7 Hz
IMU 校验错误：0
IMU 缺失样本：0
OpenVINS 单帧计算中位数：约 15.6 ms
OpenVINS 95% 单帧耗时：约 29.1 ms
```

因此，当时的主体链路不存在明显 CPU 算力不足。

办公室阶段 RTAB-Map 接受了多次有效回环，几何内点达到 `65～359`。第 66 个节点回环到第 41 个节点时获得 `173/224` 个内点，说明当时的场景识别和几何验证可以正常工作。

进入走廊以后没有继续出现旧场景回环，主要因为实际进入了新区域，而且测试结束前没有重新返回办公室。不能把“新走廊没有回环”判断为回环模块失效。

轨迹后半段约 94 秒移动约 72 米，末端距离原点约 42 米，与实际从办公室走入走廊的过程一致。没有地面真值且没有回到起点时，末端坐标大不能作为漂移证据。相同楼层上的 Z 方向累计变化约 `0.58 m`，只能说明存在一定高度漂移。

该次运行的明确异常发生在约 `17:05:02`：

- OpenVINS 停止输出；
- 约 5 秒后开始出现 `Did not receive data since 5 seconds`；
- RTAB-Map 随后因为收不到 `/odom_info` 而超时；
- 相机采集日志仍显示 14.3 FPS、零内核丢帧；
- IMU 日志仍显示约 199.5 Hz、零错误。

从 RTAB-Map ROS 源码确认，该诊断在四路双目消息成功同步并进入 `StereoOdometry::callback()` 时更新。因此可以确定，同步回调没有继续正常进入；但仅凭该次日志还不能区分以下情况：

1. 左右图像或 CameraInfo 中某一路停止被 DDS 接收；
2. 四路消息时间戳不能继续同步；
3. 下一次 OpenVINS 回调内部发生阻塞。

不能只根据这条警告就断定相机硬件停止采集。

### 9.2 18:25 最终长时间测试

昨晚最后一次运行约持续 7.5 分钟：

```text
相机：稳定约 14.30～14.31 FPS
相机发布频率：稳定约 14.30～14.31 FPS
相机内核序号丢帧：0
IMU：稳定约 199.5 Hz
IMU 错误、缺失样本和重同步：0
OpenVINS：持续输出约 6200 帧
RTAB-Map：处理到第 418 个节点
结束方式：用户 Ctrl-C
```

该次运行没有再次出现 `Did not receive data since 5 seconds`，OpenVINS 一直输出到正常退出。这说明 17:01 的同步中断是需要继续复现和加日志定位的偶发现象，不能把它当作每次必现的固定故障。

启动初期 `rtabmap_viz` 出现：

```text
Could not get odometry pose from TF
```

主要发生在 OpenVINS 完成初始化并开始发布里程计之前。传感器只预热 5 秒，不等于 OpenVINS 已初始化。若警告仅集中在启动初期并随后消失，不属于运行中丢失。

## 10. 日志排查的最简单实用方法

### 10.1 先进入日志目录

最不容易忘记的方法：

```bash
cd ~/.ros/log
ls -lt | head -n 20
```

这样可以直接看到最新修改的节点日志，不依赖 launch 目录中一定存在内容。

### 10.2 根据节点名找最新日志

```bash
ls -t stereo_odometry_*.log | head -n 1
ls -t rtabmap_*.log | head -n 1
ls -t stereo_v4l2_direct_node_*.log | head -n 1
ls -t wit_imu_node_*.log | head -n 1
```

查看错误和警告：

```bash
rg -n "ERROR|WARN|FATAL|Did not receive|Lost|lost" \
  "$(ls -t stereo_odometry_*.log | head -n 1)"
```

持续查看最新输出：

```bash
tail -f "$(ls -t stereo_odometry_*.log | head -n 1)"
```

### 10.3 使用 launch.log 找 PID

如果需要把一次运行的所有节点严格对应起来，先找对应时间的 launch 目录：

```bash
ls -td ~/.ros/log/2026-* | head
```

然后查看：

```bash
less ~/.ros/log/2026-07-23-18-25-19-*/launch.log
```

`launch.log` 中会记录每个节点的 PID，例如：

```text
stereo_odometry process started with pid [50187]
rtabmap process started with pid [50189]
```

之后直接找：

```bash
ls ~/.ros/log/*_50187_*.log
ls ~/.ros/log/*_50189_*.log
```

这种方式比单纯按文件修改时间更可靠，特别适合一天连续运行很多次时使用。

## 11. 建议采用的分层排查顺序

### 第一层：传感器是否真的稳定

相机重点看：

```text
Direct capture FPS
topic publish FPS
kernel sequence drops
```

IMU重点看：

```text
实际频率
checksum errors
missing samples
timestamp resyncs
timestamp phase error
```

如果相机和 IMU源头已经丢数据，先修传感器，暂时不要继续调 OpenVINS。

### 第二层：同步和计算是否跟得上

OpenVINS 日志重点看：

```text
update time
delay
Did not receive data since 5 seconds
初始化耗时
是否持续输出
```

判断算力不能只看 CPU 占用。只要单帧 `update time` 长期小于图像帧间隔，并且 `delay` 不持续增长，就不能简单归因于算力不足。

### 第三层：视觉前端是否健康

不能只看 `quality=0`，应检查：

```text
Odometry/Features
Odometry/LocalMapSize
图像是否模糊或过暗
快速转弯时特征是否骤降
特征是否能在重新看到纹理后恢复
```

### 第四层：RTAB-Map 是否真的形成回环

需要区分：

```text
外观候选
几何验证
回环约束被接受
图优化后轨迹是否变化
```

仅有相似度候选不等于闭环成功。必须看匹配数、几何内点和约束是否加入图中。

## 12. 源码定位方法

看到一条不理解的日志时，最有效的方法是直接搜索完整或有辨识度的报错文字：

```bash
rg -n -C 5 "Did not receive data since" \
  /home/bird/rtabmap_humble_ws/src/rtabmap_ros
```

本次由此定位到：

```text
rtabmap_odom/src/OdometryROS.cpp
rtabmap_sync/include/rtabmap_sync/SyncDiagnostic.h
stereo_odometry.cpp
```

然后沿调用关系确认：

```text
双目同步成功
  -> StereoOdometry::callback()
  -> tick(image stamp)
  -> SyncDiagnostic::tickInput()
  -> 5 秒没有下一次 tick 时打印警告
```

这种方法比根据日志字面猜测更可靠。日志通常只描述“检测到了什么”，源码才能说明检测条件和字段的真实含义。

## 13. 容易误判的现象

### 13.1 位移很大不等于漂移

如果实际走进了走廊，末端距离原点几十米是正常结果。判断漂移至少需要：

- 回到起点比较闭合误差；
- 使用轮速、动捕或其他地面真值；
- 比较已知长度路线；
- 检查静止时速度和位置是否继续变化。

### 13.2 没有回环不等于回环失效

在从未见过的新区域没有回环是正常的。测试回环必须重新经过旧位置，并尽量使用相似观察方向。

### 13.3 协方差小不等于真实误差小

OpenVINS 日志中的标准差来自滤波器自身噪声模型。如果 IMU 噪声密度设置得过小，滤波器会过度自信，即使真实轨迹已经偏离，显示的标准差也可能仍然很小。

### 13.4 Ctrl-C 后的退出错误通常不是运行故障

用户多次 Ctrl-C 后出现退出码 `-2`、`-9` 或 SIGTERM/SIGKILL，需要结合发生时间判断。只在结束阶段出现时，通常是进程被强制结束，不是运行过程中崩溃。

### 13.5 单次偶发问题不能立刻归因

17:01 测试发生同步中断，但 18:25 长时间测试未复现。正确做法是保留两次日志，找出共同条件并增加可观测量，而不是立即修改大量参数。

## 14. 当前参数风险和下一步建议

### 14.1 重新确定 IMU 噪声

当前参数为：

```text
AccelerometerNoiseDensity = 0.008
AccelerometerRandomWalk   = 0.0004
GyroscopeNoiseDensity     = 0.0001
GyroscopeRandomWalk       = 0.00001
```

陀螺仪归一化残差达到 `19.47`，说明陀螺仪噪声密度很可能明显偏小。建议优先通过 Allan 方差重新测定。

在获得正式 Allan 结果前，可单独做一组对照测试：

```text
GyroscopeNoiseDensity = 0.0015～0.0020
GyroscopeRandomWalk   = 0.0001
```

每次只改一组参数，并使用同一条路线对比，不能同时修改特征数、曝光、噪声和回环阈值。

### 14.2 时间偏移只保留一种主要补偿方式

当前相机节点已经给图像时间戳加上约 `40.946 ms` 补偿，同时参数中：

```text
OdomOpenVINS/CalibCamTimeoffset=true
```

OpenVINS 在线估计的是补偿后的剩余时偏，不是直接再次加 40.946 ms。但低纹理、运动单一时，在线时偏弱可观，可能出现不稳定估计。

建议进行 A/B 测试：

```text
A：时间戳预补偿 + CalibCamTimeoffset=false
B：时间戳预补偿 + CalibCamTimeoffset=true
```

比较相同路线、相同速度下的起终点误差和特征保持情况。

### 14.3 快速运动时控制曝光时间

当前使用硬件自动曝光：

```text
auto_exposure=3
```

暗处可能自动延长曝光，造成转弯时运动模糊。建议额外做一组手动曝光测试，把曝光限制在约 `5～10 ms`，通过增加增益补偿亮度，再比较快速转弯表现。

### 14.4 为偶发同步中断增加记录

下一次重点录制：

```text
/cam0/image_raw
/cam1/image_raw
/cam0/camera_info
/cam1/camera_info
/imu/data_raw
/odom
/odom_info
/tf
/tf_static
```

运行中同时检查：

```bash
ros2 topic hz /cam0/image_raw
ros2 topic hz /cam1/image_raw
ros2 topic hz /cam0/camera_info
ros2 topic hz /cam1/camera_info
ros2 topic hz /odom
```

只有记录四路输入的时间戳和频率，才能在偶发中断后区分发布停止、DDS接收异常、同步失败和 OpenVINS 回调阻塞。

### 14.5 使用可重复的闭环测试路线

建议固定一条测试路线：

1. 起点静止 5～10 秒；
2. 正向直行；
3. 左右横移；
4. 顺时针和逆时针转弯；
5. 经过低纹理墙面；
6. 返回起点并保持相同朝向；
7. 静止 5～10 秒后结束。

记录：

```text
起终点位置误差
起终点角度误差
最低特征数
是否出现同步超时
回环是否通过几何验证
回环前后 map->odom 是否修正
```

固定路线比“随手甩动后看起来飘不飘”更适合比较外参和参数版本。

## 15. 当日最终结论

1. 原始相机外参和矫正图像坐标轴混用的问题已经通过拆分 OpenVINS/RTAB-Map 图像输入解决。
2. 2026-07-23 的 `640×480` 双目标定几何结果总体可用，双目基线约 `50.285 mm`。
3. 新相机—IMU 外参和约 `40.946 ms` 平均时偏已经写入独立启动文件。
4. `quality=0` 和 Odometry 面板不显示角点均为当前 OpenVINS 包装层的统计/可视化限制，不能据此判断视觉已经丢失。
5. 17:01 测试出现过一次双目同步/回调中断，但 18:25 的约 7.5 分钟长测未复现；相机和 IMU源头在两次测试中都保持稳定。
6. 当前最需要继续验证的是陀螺仪噪声参数、补偿后的在线时偏估计、暗处自动曝光导致的运动模糊，以及偶发同步中断的具体位置。
7. 后续调试应使用固定闭环路线，每次只改变一个变量，并保存对应日志、数据库、参数文件和启动命令。
