# slam_ws

统一存放建图、定位和导航入口。目前由 `robot_slam_bringup` 组合传感器驱动、
OpenVINS、RTAB-Map 和 Nav2。

- `robot_slam_bringup`：当前推荐的 GO2/D435i/HB 建图导航入口；
- `stereo_slam_legacy_bringup`：从相机包迁出的历史兼容入口；
- `odom_covariance_relay`：供 EKF 和导航使用的里程计协方差转发节点。

本工作空间只保存项目自有的启动和参数。RTAB-Map、Nav2、OpenVINS 等第三方算法
继续作为外部或上游工作空间依赖，避免复制源码和维护分叉。

## 构建

先构建并加载传感器工作空间，再构建本工作空间：

```bash
source "$(git rev-parse --show-toplevel)/scripts/setup_robot_env.sh"
source /opt/ros/humble/setup.bash
source "${ROBOT_WS_ROOT}/sensor_ws/install/setup.bash"
cd "${ROBOT_WS_ROOT}/slam_ws"
colcon build --symlink-install --executor sequential
source install/setup.bash
```

也可以在仓库根目录直接执行 `./scripts/build_all.sh`。
