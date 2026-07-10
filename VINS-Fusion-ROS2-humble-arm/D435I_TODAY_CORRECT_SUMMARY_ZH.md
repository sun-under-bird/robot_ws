# D435i + Kalibr + VINS + RTAB-Map 今日正确内容总结

日期：2026-07-07

本文只记录今天最终保留下来的正确内容。已经验证不合适或已撤销的方案会单独标明，避免后续继续按错误链路排查。

文中的路径变量由 `source ~/robot_ws/scripts/setup_robot_env.sh` 设置。

## 1. 涉及工作区

VINS-Fusion ROS2 工作区：

```bash
${ROBOT_WS_ROOT}/VINS-Fusion-ROS2-humble-arm
```

RTAB-Map / camera 相关工作区：

```bash
${ROBOT_WS_ROOT}/camera
```

主要涉及包：

```text
VINS-Fusion-ROS2-humble-arm/vins
camera/src/stereo_camera_pkg
camera/src/stereo_camera_pkg_py
```

## 2. 保留的总体方案

当前正确链路是：

```text
D435i infra1/infra2 rectified image + D435i IMU
        |
        v
VINS-Fusion 使用 Kalibr 外参和内参输出 /odometry
        |
        v
RTAB-Map 使用 /odometry 作为外部里程计
        |
        v
RTAB-Map 使用 Kalibr CameraInfo topic 做双目建图
```

关键原则：

- VINS 只输出 `/odometry`，RTAB-Map 不接收 VINS 的逐帧外参。
- 不再使用“在线冻结发布外参”方案。
- RTAB-Map 的 `frame_id` 使用 `body`。
- D435i 的 RealSense TF 树需要通过静态 TF 接到 `body`。
- Kalibr 的 `T_cam_imu` 不能直接粘贴进 VINS；VINS 配置要写 `body_T_cam = inverse(T_cam_imu)`。

## 3. VINS 新 Kalibr 配置

新增文件：

```text
${ROBOT_WS_ROOT}/VINS-Fusion-ROS2-humble-arm/config/realsense_d435i/left_kalibr.yaml
${ROBOT_WS_ROOT}/VINS-Fusion-ROS2-humble-arm/config/realsense_d435i/right_kalibr.yaml
${ROBOT_WS_ROOT}/VINS-Fusion-ROS2-humble-arm/config/realsense_d435i/realsense_stereo_imu_kalibr_config.yaml
```

新 VINS 配置使用：

```yaml
cam0_calib: "left_kalibr.yaml"
cam1_calib: "right_kalibr.yaml"
image_width: 640
image_height: 480
estimate_extrinsic: 0
estimate_td: 0
td: 0.002320674987289529
```

外参来源：

```text
Kalibr: T_cam_imu
VINS:   body_T_cam
```

写入配置前已做：

```text
body_T_cam0 = inverse(cam0.T_cam_imu)
body_T_cam1 = inverse(cam1.T_cam_imu)
```

新配置复算出的双目相对外参：

```text
T_cam1_cam0 translation = [-0.049971667, 0.000012512, 0.000215181] m
baseline = 49.972 mm
```

运行 VINS 时应使用新配置：

```bash
cd ${ROBOT_WS_ROOT}/VINS-Fusion-ROS2-humble-arm
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run vins vins_node config/realsense_d435i/realsense_stereo_imu_kalibr_config.yaml
```

不建议继续用旧配置做当前标定测试：

```text
config/realsense_d435i/realsense_stereo_imu_config.yaml
```

旧配置里的外参和新 Kalibr 结果相差约：

```text
cam0: 平移约 13.41 mm，旋转约 0.817 deg
cam1: 平移约 13.48 mm，旋转约 0.778 deg
```

这个差距足够影响 VINS 稳定性。

## 4. Kalibr CameraInfo 发布节点

RealSense 默认 `/camera_info` 不能直接通过 launch 参数改成自定义 Kalibr 内参。今天采取的正确方式是新建一个独立节点，订阅图像并按图像时间戳发布新的 CameraInfo。

新增文件：

```text
${ROBOT_WS_ROOT}/camera/src/stereo_camera_pkg/src/d435i_kalibr_camera_info_node.cpp
${ROBOT_WS_ROOT}/camera/src/stereo_camera_pkg/config/d435i_infra1_kalibr_camera_info.yaml
${ROBOT_WS_ROOT}/camera/src/stereo_camera_pkg/config/d435i_infra2_kalibr_camera_info.yaml
${ROBOT_WS_ROOT}/camera/src/stereo_camera_pkg/launch/d435i_kalibr_camera_info.launch.py
```

修改文件：

```text
${ROBOT_WS_ROOT}/camera/src/stereo_camera_pkg/CMakeLists.txt
${ROBOT_WS_ROOT}/camera/src/stereo_camera_pkg/package.xml
```

节点默认订阅：

```text
/camera/camera/infra1/image_rect_raw
/camera/camera/infra2/image_rect_raw
```

节点默认发布：

```text
/camera/camera/infra1/camera_info_kalibr
/camera/camera/infra2/camera_info_kalibr
```

右目 CameraInfo 的投影矩阵中：

```text
P[3] = -19.452999453366537
```

对应 baseline：

```text
baseline = -P[3] / P[0] = 0.049971667 m
```

编译：

```bash
cd ${ROBOT_WS_ROOT}/camera
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select stereo_camera_pkg
```

启动：

```bash
source /opt/ros/humble/setup.bash
source ${ROBOT_WS_ROOT}/camera/install/setup.bash

ros2 launch stereo_camera_pkg d435i_kalibr_camera_info.launch.py
```

检查：

```bash
ros2 topic echo /camera/camera/infra1/camera_info_kalibr --once
ros2 topic echo /camera/camera/infra2/camera_info_kalibr --once
```

注意：不要把这个节点发布到 RealSense 原始话题：

```text
/camera/camera/infra1/camera_info
/camera/camera/infra2/camera_info
```

否则会和 RealSense 自带 publisher 冲突。

## 5. RTAB-Map 启动文件修正

修改文件：

```text
${ROBOT_WS_ROOT}/camera/src/stereo_camera_pkg_py/launch/d435i.launch.py
```

保留的关键修正：

1. 增加静态 TF：

```text
body -> camera_link
```

对应参数：

```text
x = -0.00552
y =  0.00510
z =  0.01174
qx =  0.5
qy = -0.5
qz =  0.5
qw =  0.5
```

这个静态 TF 的作用是把 VINS 的 `body` 坐标系和 RealSense 的 `camera_link -> camera_infra*_optical_frame` TF 树接起来。

2. RTAB-Map 主节点使用：

```python
'frame_id': 'body'
'subscribe_stereo': True
'subscribe_odom_info': False
'approx_sync': True
'approx_sync_max_interval': 0.1
'Rtabmap/ImagesAlreadyRectified': 'true'
```

3. `rtabmap_viz` 也使用：

```python
'frame_id': 'body'
'approx_sync_max_interval': 0.1
'wait_for_transform': 0.5
```

4. RTAB-Map remap 到 Kalibr CameraInfo：

```python
('left/image_rect', '/camera/camera/infra1/image_rect_raw')
('right/image_rect', '/camera/camera/infra2/image_rect_raw')
('left/camera_info', '/camera/camera/infra1/camera_info_kalibr')
('right/camera_info', '/camera/camera/infra2/camera_info_kalibr')
('odom', '/odometry')
```

编译安装 launch 更新：

```bash
cd ${ROBOT_WS_ROOT}/camera
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select stereo_camera_pkg_py
```

启动：

```bash
source /opt/ros/humble/setup.bash
source ${ROBOT_WS_ROOT}/camera/install/setup.bash

ros2 launch stereo_camera_pkg_py d435i.launch.py
```

## 6. RealSense 正确启动方式

必须使用：

```text
depth_module.infra_profile:=640x480x30
```

不要使用：

```text
infra_width
infra_height
infra_fps
```

当前 RealSense ROS2 驱动会忽略 `infra_width/infra_height/infra_fps`，导致实际输出可能变成 `848x480`，从而和 VINS/Kalibr 的 `640x480` 内参不匹配。

推荐启动：

```bash
source /opt/ros/humble/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_depth:=false \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  enable_sync:=true \
  depth_module.infra_profile:=640x480x30
```

检查频率：

```bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/infra2/image_rect_raw
ros2 topic hz /camera/camera/imu
```

正常参考值：

```text
infra1: 约 29-30 Hz
infra2: 约 29-30 Hz
imu:    约 199-200 Hz
```

## 7. 推荐完整启动顺序

终端 1：启动 RealSense。

```bash
source /opt/ros/humble/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_depth:=false \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  enable_sync:=true \
  depth_module.infra_profile:=640x480x30
```

终端 2：发布 Kalibr CameraInfo。

```bash
source /opt/ros/humble/setup.bash
source ${ROBOT_WS_ROOT}/camera/install/setup.bash

ros2 launch stereo_camera_pkg d435i_kalibr_camera_info.launch.py
```

终端 3：启动 VINS。

```bash
cd ${ROBOT_WS_ROOT}/VINS-Fusion-ROS2-humble-arm
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run vins vins_node config/realsense_d435i/realsense_stereo_imu_kalibr_config.yaml
```

终端 4：启动 RTAB-Map。

```bash
source /opt/ros/humble/setup.bash
source ${ROBOT_WS_ROOT}/camera/install/setup.bash

ros2 launch stereo_camera_pkg_py d435i.launch.py
```

启动后检查：

```bash
ros2 topic hz /odometry
ros2 topic echo /camera/camera/infra1/camera_info_kalibr --once
ros2 topic echo /camera/camera/infra2/camera_info_kalibr --once
ros2 run tf2_ros tf2_echo body camera_infra1_optical_frame
```

`tf2_echo body camera_infra1_optical_frame` 应能查到 transform。如果报两棵 TF 树不连通，说明 `body -> camera_link` 静态 TF 没启动，或者重复启动不同版本的 TF bridge。

## 8. RealSense 超时恢复

今天遇到的错误：

```text
Frames didn't arrived within 5 seconds
iio_hid_sensor: Frames didn't arrived within the predefined interval
UVCIOC_CTRL_QUERY failed on control 1 Last Error: Connection timed out
```

这个是 D435i/librealsense 设备卡死，不是 VINS 或 RTAB-Map 算法问题。

先检查是否还能枚举设备：

```bash
rs-enumerate-devices
```

如果 `lsusb` 还能看到 `8086:0b3a`，但 `rs-enumerate-devices` 看不到设备，执行：

```bash
sudo usbreset 8086:0b3a
```

今天实测 `usbreset` 后恢复正常：

```text
infra1: 约 29 Hz
imu:    约 199.7 Hz
```

如果 `usbreset` 后仍失败，需要在虚拟机 USB 直通里断开再连接 D435i，或者物理重插。

## 9. TF 注意事项

RealSense 自己会发布：

```text
camera_link -> camera_infra1_optical_frame
camera_link -> camera_infra2_optical_frame
camera_link -> camera_imu_optical_frame
```

VINS / RTAB-Map 使用：

```text
body
```

所以必须有：

```text
body -> camera_link
```

今天已把这个桥接加入：

```text
${ROBOT_WS_ROOT}/camera/src/stereo_camera_pkg_py/launch/d435i.launch.py
```

不要同时启动多个不同来源的 `body -> camera_link` 静态 TF。例如，如果已经启动 `stereo_camera_pkg_py d435i.launch.py`，就不要再同时启动 VINS 里的 `d435i_vins_rviz.launch.xml` 中同名桥接，除非确认两个参数完全一致且不会重复发布。

## 10. 已撤销的内容

今天曾尝试过“VINS 在线优化外参后冻结发布给 RTAB-Map”的方案，最终撤销。

撤销后当前状态：

```text
不再发布 /vins/extrinsic/*
不再发布 /calibration/*
不再有 calibration_manager 节点
```

验证过：

```bash
ros2 pkg executables vins
```

只剩：

```text
vins vins_node
```

撤销原因：

- VINS 在线外参估计是滑窗优化值，不适合直接作为 RTAB-Map 每帧动态 CameraInfo。
- 实测冻结结果不够可信。
- 当前更稳的做法是离线 Kalibr 标定后固定使用。

## 11. 里程计突然飞远的重点排查

如果 `/odometry` 突然跳到很远，优先按下面顺序查：

1. D435i 是否掉流：

```bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/infra2/image_rect_raw
ros2 topic hz /camera/camera/imu
```

2. RealSense 终端是否出现：

```text
Frames didn't arrived within 5 seconds
UVCIOC_CTRL_QUERY failed
iio_hid_sensor timeout
```

有则先：

```bash
sudo usbreset 8086:0b3a
```

3. VINS 终端是否频繁出现：

```text
throw img0
throw img1
n_pts size 很小
failure detection
```

4. 是否混用了旧 VINS 配置：

```text
错误倾向：config/realsense_d435i/realsense_stereo_imu_config.yaml
当前推荐：config/realsense_d435i/realsense_stereo_imu_kalibr_config.yaml
```

5. 虚拟机负载是否过高。

建议先只跑 RealSense + VINS，确认 `/odometry` 稳定后再启动 RTAB-Map 和 `rtabmap_viz`。

## 12. 当前仍需注意的技术边界

Kalibr 结果中有非零畸变参数，但当前图像 topic 是：

```text
/camera/camera/infra1/image_rect_raw
/camera/camera/infra2/image_rect_raw
```

`image_rect_raw` 通常表示已经由 RealSense 做过 rectification。严格来说，如果图像已经矫正，CameraInfo 里的畸变应接近 0。

当前保留这种配置的原因：

- 你提供的 Kalibr YAML 里 rostopic 写的就是 `image_rect_raw`。
- 所以今天按你给出的 rostopic 生成了对应配置。

如果后续确认 Kalibr 实际使用的是未矫正 raw 图像，则要改为使用未矫正图像 topic，并让 VINS / RTAB-Map 使用与标定一致的相机模型。

## 13. 快速健康检查命令

设备：

```bash
lsusb | grep 8086
rs-enumerate-devices
```

RealSense 话题：

```bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/infra2/image_rect_raw
ros2 topic hz /camera/camera/imu
```

Kalibr CameraInfo：

```bash
ros2 topic echo /camera/camera/infra1/camera_info_kalibr --once
ros2 topic echo /camera/camera/infra2/camera_info_kalibr --once
```

VINS：

```bash
ros2 topic hz /odometry
ros2 topic echo /odometry --once
```

TF：

```bash
ros2 run tf2_ros tf2_echo body camera_infra1_optical_frame
ros2 run tf2_ros tf2_echo body camera_infra2_optical_frame
```

RTAB-Map 同步失败时，先看：

```text
Did not receive data since 5 seconds
TF of received image is not set
Could not find a connection between body and camera_infra1_optical_frame
```

分别对应：

```text
话题时间同步问题
TF bridge 没启动
RealSense/VINS/CameraInfo 某一路没有发布
```
