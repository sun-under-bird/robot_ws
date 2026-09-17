# sensor_ws

统一存放机器人传感器驱动和传感器数据预处理包。

## 包说明

- `stereo_v4l2_camera`：唯一保留的 USB 拼接双目相机 C++ V4L2 驱动。
- `wit_imu`：当前 WIT 串口 IMU 的 C++ 驱动及 Python 回退节点。

原先混在相机包中的 RTAB-Map、OpenVINS、EKF 和 Nav2 文件已迁移到
`slam_ws/src/stereo_slam_legacy_bringup`。不要继续向传感器包增加 SLAM 启动文件。

## 构建

```bash
source "$(git rev-parse --show-toplevel)/scripts/setup_robot_env.sh"
cd "${ROBOT_WS_ROOT}/sensor_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential
source install/setup.bash
```

## 相机启动

驱动包只保留一个完整模板，默认使用低算力的 HB YUYV 1280×480@15 FPS：

```bash
ros2 launch stereo_v4l2_camera stereo_v4l2_camera.launch.py
```

标定文件不再存放于驱动包中。需要标定 `CameraInfo` 时，通过
`left_camera_info_file` 和 `right_camera_info_file` 传入外部 YAML 绝对路径。
