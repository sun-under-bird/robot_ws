# person_3d_localization

接收上层给出的单个人体 bbox，从与原图时间戳匹配的对齐深度图中提取稳健三维均值点，
然后在 `map` 坐标系中围绕人体生成候选目标，用 Nav2 全局代价地图剔除障碍/未知区域，
再调用 `/compute_path_to_pose` 验证路径，返回第一个安全且可达的固定目标。包内不包含
YOLO、人体跟踪或自动导航；`/navigate_to_pose` 仍由上层调用。

完整的上层请求字段、响应字段、错误码和调用示例见
[`docs/UPPER_LAYER_INTERFACE.md`](docs/UPPER_LAYER_INTERFACE.md)。

## 数据约束

- bbox 必须是 YOLO 缩放和 letterbox 还原后的原图坐标。
- bbox 所属图像必须与 `depth_topic` 使用同一像素坐标系；D435i 推荐使用对齐到彩色图的深度。
- 请求应携带 bbox 原图的时间戳。零时间戳仅用于人工测试，会选择最新且未过期的深度帧。
- 默认缓存 150 帧深度和 CameraInfo，60 Hz 时约覆盖 2.5 秒上层处理延迟。
- `16UC1` 深度默认按毫米解释，比例由 `depth_16u_scale` 控制；`32FC1` 按米解释。
- 需要运行中的 Nav2 全局代价地图和规划器；若机器人已在安全距离内，则无需路径规划。
- 默认采样 16 个方向、3 层半径（1.5、1.8、2.1 米），每次请求最多规划 48 个候选点，
  搜索总时限为 20 秒。候选点必须位于已知安全区域，且可规划到达。

## 构建与启动

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_ws_leg_velocity/slam_ws
colcon build --packages-select person_3d_localization --symlink-install
source install/setup.bash
ros2 launch person_3d_localization person_3d_localization.launch.py
```

D435i 必须发布与 bbox 图像对齐的深度，例如默认配置使用：

```text
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

## 服务

服务名称：`/person_3d_localization/locate_from_bbox`

上层传入 bbox、原图时间戳和可选安全距离。`stand_off_distance <= 0` 时使用参数中的
`default_stand_off_distance`。成功响应包含人体 `person_point` 和已验证可达的固定目标
`navigation_goal`；上层只需把后者作为一次 `NavigateToPose` action 目标发送给现有 Nav2。
若代价地图、规划器不可用或没有可达候选点，服务返回失败，上层不得发送目标。

人工测试可使用零时间戳选取最新深度：

```bash
ros2 service call /person_3d_localization/locate_from_bbox \
  person_3d_localization/srv/LocateFromBbox \
  "{image_header: {frame_id: '', stamp: {sec: 0, nanosec: 0}}, \
    bbox: {x_offset: 200, y_offset: 100, width: 180, height: 300, do_rectify: false}, \
    stand_off_distance: 1.5}"
```

调试输出：

- `/person_3d_localization/person_point`
- `/person_3d_localization/navigation_goal`

启动导航时需使用 `ros2 launch robot_slam_bringup nav.launch.py use_nav2:=true`；现有
固定目标行为树无需修改。路径验证只反映请求时的地图状态；后续出现动态障碍时，
Nav2 仍可能返回导航失败。
