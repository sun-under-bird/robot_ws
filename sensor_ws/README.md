# sensor_ws

统一存放机器人传感器驱动和传感器数据预处理包。

## 包说明

- `stereo_v4l2_camera`：当前 HB USB 双目相机的 C++ V4L2 驱动。
- `wit_imu`：当前 WIT 串口 IMU 的 C++ 驱动及 Python 回退节点。
- `stereo_camera_pkg`：C++ 相机采集、切分、标定信息和预处理节点。
- `stereo_camera_pkg_py`：Python 相机采集、切分和标定工具。
- `stereo_cam`：历史 Python 拼接双目切分节点。

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
