# 上层 bbox 单次人体定位接口说明

## 1. 功能边界

`person_3d_localization` 不运行 YOLO，也不直接发送导航目标。它在一次服务请求中
完成深度定位、候选点搜索和路径验证：

```text
上层保留检测图像 Header
        ↓
上层运行 YOLO 并选定一个人体 bbox
        ↓
调用 /person_3d_localization/locate_from_bbox
        ↓
本节点匹配同一时刻的对齐深度图和 CameraInfo
        ↓
bbox 深度过滤 → 相机三维均值点 → TF 转换到 map
        ↓
人体周围候选点 → 全局代价地图安全过滤 → Nav2 路径规划验证
        ↓
返回人体点和已验证可达的固定导航目标
        ↓
上层决定是否发送一次 NavigateToPose
```

每次服务请求只处理一个 bbox。多人场景下，由上层先选择要跟随的人，再传入该人的
bbox。本节点不进行人体 ID 关联、连续跟踪或目标更新。
同一次跟随任务，上层应只调用一次定位服务、发送一次导航目标；ID 的后续刷新不应
重复触发服务请求。只有业务明确要求重新定位时才重新调用。

## 2. 通信接口总览

| 方向 | 名称 | ROS 2 类型 | 用途 |
|---|---|---|---|
| 上层 → 本节点 | `/person_3d_localization/locate_from_bbox` | `person_3d_localization/srv/LocateFromBbox` | 提交一次 bbox 定位请求 |
| 相机 → 本节点 | `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | 对齐到检测图像的深度图 |
| 相机 → 本节点 | `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 检测图像对应的相机内参 |
| Nav2 → 本节点 | `/global_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | 已知区域、障碍和候选目标安全性 |
| 本节点 → Nav2 | `/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | 请求候选目标的全局路径，验证可达性 |
| 本节点 → 调试 | `/person_3d_localization/person_point` | `geometry_msgs/msg/PointStamped` | 成功定位后的人体三维均值点 |
| 本节点 → 调试 | `/person_3d_localization/navigation_goal` | `geometry_msgs/msg/PoseStamped` | 成功计算后的固定导航目标 |
| 上层 → Nav2 | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 上层按需发送一次导航目标 |

服务名称、话题和规划 Action 名称都可以在参数文件中修改。深度图和 CameraInfo 使用
`SensorDataQoS`；服务使用 ROS 2 请求/响应机制。

## 3. 上层必须提供的数据

服务定义：

```text
std_msgs/Header image_header
sensor_msgs/RegionOfInterest bbox
float32 stand_off_distance
---
bool success
bool navigation_required
uint8 status
string message
geometry_msgs/PointStamped person_point
geometry_msgs/PoseStamped navigation_goal
float32 valid_depth_ratio
float32 mean_depth
float32 depth_stddev
```

### 3.1 `image_header`

必须使用产生该 bbox 的原始相机图像 Header，尤其是原始时间戳：

| 字段 | 要求 |
|---|---|
| `image_header.stamp` | 复制检测输入图像的原始时间戳，不能填写服务调用时刻 |
| `image_header.frame_id` | 建议复制检测输入图像的 frame，默认通常为 `camera_color_optical_frame` |

YOLO 推理结束后，深度相机可能已经发布了后续帧。本节点缓存最近的深度帧，并使用
`image_header.stamp` 找到和 bbox 对应的那一帧，而不是直接使用收到请求时的最新深度。

默认允许的图像与深度时间差为 `0.05 s`。缓存长度为 150 帧，30 Hz 时约覆盖
5 秒，60 Hz 时约覆盖 2.5 秒，用于保守覆盖约 2 秒的上层处理延迟。缓存按帧数
限制，实际可回溯时间取决于深度图发布帧率；如高于 60 Hz 或处理时间更长，
需要增大 `cache_size`。上层 ID 刷新不影响匹配，服务只依据原图时间戳查找深度帧。
请求到达时对应深度帧必须已经发布，缓存不会等待未来帧。

零时间戳只用于人工调试：节点会选择最新且不超过 `max_latest_depth_age` 的深度帧。
正式上层程序不得依赖零时间戳。

### 3.2 `bbox`

类型为 `sensor_msgs/msg/RegionOfInterest`：

| 字段 | 单位 | 含义 |
|---|---:|---|
| `x_offset` | pixel | bbox 左上角横坐标 |
| `y_offset` | pixel | bbox 左上角纵坐标 |
| `width` | pixel | bbox 宽度，必须大于 0 |
| `height` | pixel | bbox 高度，必须大于 0 |
| `do_rectify` | — | 当前实现不使用，填写 `false` |

bbox 坐标必须满足以下要求：

1. 已经从 YOLO 网络输入尺寸还原到原始检测图像尺寸；
2. 已经撤销 letterbox padding；
3. 与配置的深度图处于同一个像素坐标系；
4. D435i 默认配置要求 bbox 来自彩色图，深度为 aligned-to-color 深度；
5. 坐标使用左上角为原点，`u` 向右、`v` 向下。

例如，YOLO 在 `640×640` letterbox 图上给出的 bbox，不能直接用于一幅
`640×480` 原图。上层必须先撤销上下 padding，恢复到 `640×480` 坐标。

### 3.3 `stand_off_distance`

导航目标与人体之间的安全距离，单位为米：

- 大于 `0`：使用本次请求值；
- 小于或等于 `0`：使用节点参数 `default_stand_off_distance`；
- 默认值：`1.5 m`。

目标点在 `map` 平面内按以下关系搜索：

```text
base_angle = angle(robot_xy - person_xy)
candidate_xy = person_xy + radius × [cos(angle), sin(angle)]
radius = stand_off_distance + level × radial_step
goal_yaw = 朝向 person_xy
```

默认优先尝试原来的“机器人与人连线”目标，再交替尝试左右方向。默认 16 个方向、
3 层半径（`stand_off_distance`、再外移 0.3 米和 0.6 米）；不会把目标放得比请求距离
更靠近人体。每个候选点都要通过全局代价地图检查与 Nav2 路径规划，最终位置可能不在
最初的连线上。`stand_off_distance` 是名义目标半径；Nav2 的规划和到达容差会使
机器人最终停车位置与该半径略有偏差，不能把它当作严格的物理安全边界。

## 4. 请求示例

### 4.1 命令行请求

正式请求应使用检测图像的真实时间戳：

```bash
ros2 service call /person_3d_localization/locate_from_bbox \
  person_3d_localization/srv/LocateFromBbox \
  "{image_header: {
       stamp: {sec: 1789700000, nanosec: 123456789},
       frame_id: 'camera_color_optical_frame'},
     bbox: {
       x_offset: 210,
       y_offset: 90,
       width: 170,
       height: 320,
       do_rectify: false},
     stand_off_distance: 1.5}"
```

零时间戳调试请求：

```bash
ros2 service call /person_3d_localization/locate_from_bbox \
  person_3d_localization/srv/LocateFromBbox \
  "{image_header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
     bbox: {x_offset: 210, y_offset: 90, width: 170, height: 320, do_rectify: false},
     stand_off_distance: 1.5}"
```

### 4.2 Python 上层请求

```python
from person_3d_localization.srv import LocateFromBbox

# image_msg 是送入 YOLO 的原始 ROS 图像消息，bbox 坐标已还原到它的分辨率。
request = LocateFromBbox.Request()
request.image_header = image_msg.header
request.bbox.x_offset = int(x1)
request.bbox.y_offset = int(y1)
request.bbox.width = int(x2 - x1)
request.bbox.height = int(y2 - y1)
request.bbox.do_rectify = False
request.stand_off_distance = 1.5

client = node.create_client(
    LocateFromBbox,
    '/person_3d_localization/locate_from_bbox',
)
future = client.call_async(request)
```

应在 `future` 完成回调中先判断 `response.success`，再判断
`response.navigation_required`；包含 Nav2 Action 发送及结果处理的示例见第 8 节。

### 4.3 C++ 上层请求

```cpp
auto request = std::make_shared<person_3d_localization::srv::LocateFromBbox::Request>();
// 必须保留产生 bbox 的原图时间戳。
request->image_header = image_message->header;
request->bbox.x_offset = static_cast<uint32_t>(x1);
request->bbox.y_offset = static_cast<uint32_t>(y1);
request->bbox.width = static_cast<uint32_t>(x2 - x1);
request->bbox.height = static_cast<uint32_t>(y2 - y1);
request->bbox.do_rectify = false;
request->stand_off_distance = 1.5F;

auto future = locate_client->async_send_request(request);
```

## 5. 本节点返回的数据

### 5.1 通用状态字段

| 字段 | 含义 |
|---|---|
| `success` | 整次定位和目标计算是否成功 |
| `navigation_required` | 是否需要把 `navigation_goal` 发送给 Nav2 |
| `status` | 结构化状态码 |
| `message` | 中文状态说明，主要用于日志和调试 |

上层处理规则：

```text
success == false
    丢弃 person_point 和 navigation_goal，根据 status 处理错误

success == true && navigation_required == false
    人体定位成功，但机器人已经在安全距离内，不发送平移导航目标

success == true && navigation_required == true
    将 navigation_goal 发送一次给 NavigateToPose
```

### 5.2 `person_point`

`geometry_msgs/msg/PointStamped`，表示 bbox 内稳健深度点反投影后的三维均值：

- `header.frame_id`：默认是 `map`；由参数 `target_frame` 决定；
- `header.stamp`：匹配到的深度测量时间戳；
- `point.x/y/z`：人体均值点在目标坐标系中的位置，单位为米。

该点通常接近人体躯干区域的三维均值，不是人体脚底点。导航只使用它的平面 `x/y`。

### 5.3 `navigation_goal`

`geometry_msgs/msg/PoseStamped`，可以直接赋给 `NavigateToPose.Goal.pose`：

- `header.frame_id`：默认是 `map`；
- `header.stamp`：目标生成时刻；
- `pose.position.x/y`：人体周围经过安全性和可达性验证的固定候选点；
- `pose.position.z`：固定为 `0`；
- `pose.orientation`：使机器人朝向人体的四元数。

当 `navigation_required=false` 时，目标位置会填写为测量时刻机器人自身位置。上层默认
不应发送该目标；如果产品逻辑希望机器人原地朝向人体，可以自行决定是否使用其朝向。
当 `navigation_required=true` 时，成功只保证请求时的代价地图和规划器认为目标可达；
地图或动态障碍随后改变，最终导航仍可能失败，上层仍须处理 Nav2 Action 结果。

### 5.4 深度质量字段

| 字段 | 单位 | 含义 |
|---|---:|---|
| `valid_depth_ratio` | 0～1 | bbox 中央采样区域经深度过滤后保留的比例 |
| `mean_depth` | m | 内点在相机光轴方向上的平均深度 Z |
| `depth_stddev` | m | 内点深度标准差，越小通常越稳定 |

`mean_depth` 是相机光学坐标系的 Z，不是人体到机器人的平面欧氏距离。

## 6. 状态码

| 数值 | 常量 | 含义 | 上层建议 |
|---:|---|---|---|
| 0 | `STATUS_OK` | 成功 | 根据 `navigation_required` 决定是否导航 |
| 1 | `STATUS_NO_DEPTH` | 没收到深度，或缓存中没有匹配时间戳的深度 | 检查 aligned depth 和时间戳，必要时稍后重试 |
| 2 | `STATUS_NO_CAMERA_INFO` | 没有匹配内参，或内参与深度分辨率不一致 | 检查 CameraInfo 话题和相机 profile |
| 3 | `STATUS_INVALID_BBOX` | bbox 为空或完全越界 | 检查 bbox 还原和裁剪逻辑 |
| 4 | `STATUS_INVALID_DEPTH` | 有效深度不足、离群过滤后点不足或几何结果异常 | 不导航，可重新检测或提示失败 |
| 5 | `STATUS_TF_UNAVAILABLE` | 测量时刻无法转换到目标坐标系 | 检查 TF 链和时间同步 |
| 6 | `STATUS_INVALID_REQUEST` | 安全距离、frame 或其他请求数据非法 | 修正上层请求 |
| 7 | `STATUS_NAV2_UNAVAILABLE` | 全局代价地图缺失/过期，或规划 Action 不可用/拒绝请求 | 检查 Nav2、地图和时间同步；稍后重试 |
| 8 | `STATUS_NO_REACHABLE_GOAL` | 所有候选点均被地图过滤，或可用候选点均无路径 | 不发送导航；检查地图覆盖、障碍和安全距离 |
| 9 | `STATUS_GOAL_SEARCH_TIMEOUT` | 路径规划等待、候选数量或总搜索时限已达到上限 | 不发送导航；检查规划器负载或调整搜索参数 |

失败响应中的点和目标保持默认值，上层不得在 `success=false` 时使用它们。

## 7. TF 与坐标系要求

默认参数：

```yaml
target_frame: map
robot_frame: base_footprint
```

运行时必须存在测量时刻的完整 TF 链：

```text
map → odom → base_footprint → camera_color_optical_frame
```

节点使用深度图的 `header.frame_id` 作为三维点源坐标系，并使用深度测量时间戳查询 TF。
它不会使用“最新 TF”替代历史测量时刻的 TF。

## 8. 发送给 Nav2

`person_3d_localization` **会调用 Nav2 规划 Action 验证目标，但不会自动发起导航**。
导航模式下启动
`robot_slam_bringup/nav.launch.py` 时必须传入 `use_nav2:=true`（该参数默认是
`false`）；建图或定位模式等其他参数仍按现场部署选择。上层还需确认
`/navigate_to_pose` Action 服务器已就绪；本节点还需要 `/compute_path_to_pose` 和
`/global_costmap/costmap_raw`。服务可能等待多个候选点的规划结果，默认最多 20 秒，
再加上 TF、图像处理和规划器连接开销；建议上层服务客户端超时至少设为 25 秒。

上层只在以下条件同时满足时发送导航：

```text
response.success == true
response.navigation_required == true
response.navigation_goal.header.frame_id == "map"
```

下面接续第 4.2 节的 `node` 和 `future`。在上层节点初始化时创建一次
`ActionClient`，把定位服务响应回调挂到 `future` 上。上层 ROS 包需声明
`rclpy`、`person_3d_localization`、`nav2_msgs` 和 `action_msgs` 依赖：

```python
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

nav_action_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')


def on_navigation_result(done_future):
    """记录单次导航的最终状态。"""
    try:
        result = done_future.result()
    except Exception as exc:
        node.get_logger().error(f'获取 Nav2 结果失败: {exc}')
        return
    if result.status == GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().info('已到达固定目标点')
    else:
        node.get_logger().warning(f'导航未成功，状态码: {result.status}')


def on_goal_response(done_future):
    """确认目标是否被 Nav2 接受，并等待最终结果。"""
    try:
        goal_handle = done_future.result()
    except Exception as exc:
        node.get_logger().error(f'发送 Nav2 目标失败: {exc}')
        return
    if not goal_handle.accepted:
        node.get_logger().warning('Nav2 拒绝了导航目标')
        return
    goal_handle.get_result_async().add_done_callback(on_navigation_result)


def on_localization_response(done_future):
    """检查定位响应，必要时向 Nav2 发送一次固定目标。"""
    try:
        response = done_future.result()
    except Exception as exc:
        node.get_logger().error(f'人体定位服务调用失败: {exc}')
        return
    if not response.success:
        node.get_logger().error(
            f'人体定位失败: status={response.status}, message={response.message}'
        )
        return
    if not response.navigation_required:
        node.get_logger().info('机器人已经位于安全距离内，不发送导航目标')
        return
    if response.navigation_goal.header.frame_id != 'map':
        node.get_logger().error('导航目标不在 map 坐标系，拒绝发送')
        return
    if not nav_action_client.server_is_ready():
        node.get_logger().error('/navigate_to_pose 不可用，请检查 use_nav2:=true')
        return

    goal = NavigateToPose.Goal()
    goal.pose = response.navigation_goal
    nav_action_client.send_goal_async(goal).add_done_callback(on_goal_response)


future.add_done_callback(on_localization_response)
```

这是一轮服务请求对应最多一次固定目标导航。本节点不会继续更新目标，上层应保持
`rclpy` executor 运行以处理异步回调。调试话题
`/person_3d_localization/navigation_goal` 不是自动导航接口，不能只订阅或发布该话题
就认为 Nav2 已开始运动。

## 9. 启动和联调检查

启动节点：

```bash
source /opt/ros/humble/setup.bash
source /home/bird/robot_ws_leg_velocity/slam_ws/install/setup.bash
ros2 launch person_3d_localization person_3d_localization.launch.py
```

调用服务前依次检查：

```bash
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic hz /camera/camera/color/camera_info
ros2 topic hz /global_costmap/costmap_raw
ros2 topic echo /camera/camera/aligned_depth_to_color/image_raw --once --field header
ros2 service type /person_3d_localization/locate_from_bbox
ros2 action list -t
ros2 run tf2_ros tf2_echo map camera_color_optical_frame
ros2 run tf2_ros tf2_echo map base_footprint
```

期望的服务类型：

```text
person_3d_localization/srv/LocateFromBbox
```

需要导航时，`ros2 action list -t` 中还应出现
`/compute_path_to_pose [nav2_msgs/action/ComputePathToPose]` 和
`/navigate_to_pose [nav2_msgs/action/NavigateToPose]`。

成功调用后还可以在 RViz 或命令行检查：

```bash
ros2 topic echo /person_3d_localization/person_point --once
ros2 topic echo /person_3d_localization/navigation_goal --once
```

## 10. 默认参数摘要

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `default_stand_off_distance` | 1.5 | 默认人体安全距离，米 |
| `arrival_tolerance` | 0.15 | 已处于安全距离的附加容差，米 |
| `cache_size` | 150 | 深度和 CameraInfo 缓存帧数；60 Hz 时约覆盖 2.5 秒 |
| `angular_samples` | 16 | 围绕人体采样的角度数量 |
| `radial_levels` / `radial_step` | 3 / 0.3 | 从请求安全距离起向外采样的层数和米数 |
| `clearance_radius` / `max_goal_cost` | 0.45 / 199 | 候选点安全检查的半径和最大允许代价 |
| `max_costmap_age` | 3.0 | 最大全局代价地图年龄，秒 |
| `candidate_plan_timeout` | 3.0 | 单个候选点等待 Nav2 结果的时限，秒 |
| `candidate_search_timeout` | 20.0 | 整次候选点搜索总时限，秒 |
| `path_endpoint_tolerance` | 0.25 | 规划路径末端与候选目标允许的最大偏差，米 |
| `max_planning_candidates` | 48 | 单次请求最多向 Nav2 查询的候选点数量 |
| `max_timestamp_difference` | 0.05 | bbox 图像与深度最大时间差，秒 |
| `depth_min` | 0.3 | 最小有效深度，米 |
| `depth_max` | 6.0 | 最大有效深度，米 |
| `bbox_width_scale` | 0.6 | bbox 中央采样区域宽度比例 |
| `bbox_height_scale` | 0.7 | bbox 中央采样区域高度比例 |
| `min_valid_depth_ratio` | 0.20 | 最低有效深度比例 |
| `tf_timeout` | 0.3 | 单次 TF 查询超时，秒 |

修改默认参数请编辑 `config/person_3d_localization.yaml`，或通过自定义参数文件启动。
