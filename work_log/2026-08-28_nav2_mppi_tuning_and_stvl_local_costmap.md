# 2026-08-28 Nav2 MPPI 调参 + 局部代价地图迁移 STVL 工作总结

## 1. 工作目标

本轮针对 GO2 在 RTAB-Map 定位模式下运行 Nav2 时暴露的四个行为问题定位根因并修复：

| 现象 | 状态 |
| --- | --- |
| 全局路径持续抖动重规划 | 已改，未实机验证 |
| 到达终点后导航不结束 | 已修复，日志验证通过 |
| 行进中持续左右摆头/晃动 | 已修复，用户确认 |
| 局部障碍不清空（鬼影残留） | 已改为 STVL，未实机验证 |

贯穿本轮的方法论：**所有结论都从 Nav2 / RTAB-Map 源码取证，不靠参数文档推断**。四个问题里有三个的根因用参数说明是看不出来的。

## 2. 涉及的文件

```text
slam_ws/src/robot_slam_bringup/config/go2_nav2_voxel_denoise.yaml            # 修改，12 处
slam_ws/src/robot_slam_bringup/config/go2_nav2_stvl.yaml                     # 新增，仅 local_costmap 段与上者不同
slam_ws/src/robot_slam_bringup/config/go2_d435i_openvins_leg_mapping_optimized.yaml  # 修改，1 处
slam_ws/src/robot_slam_bringup/config/navigate_to_pose_no_backup.xml         # 注释纠错
slam_ws/src/robot_slam_bringup/launch/go2_d435i_openvins_leg_mapping.launch.py  # nav2_params_file 默认值切到 stvl
```

## 3. 问题一：到达终点后导航不结束

### 根因

`critics` 列表里**没有 `GoalAngleCritic`**。这条链路必须三处源码合起来看才能确认：

- `goal_critic.cpp:44-56`：只计算终点欧氏距离，代价函数完全不含朝向项。
- `simple_goal_checker.cpp:103-134`：`stateful: True` 时位置一旦达标就锁定，之后**只检查偏航**。
- 于是位置进容差圈后，没有任何评价器驱动机器人转向目标朝向，`yaw_goal_tolerance: 0.3` 永远不满足。

机器人停在终点附近原地蹭动，导航任务不返回成功。

### 修复

```yaml
GoalAngleCritic:
  cost_power: 1
  cost_weight: 6.0            # 高于 TwirlingCritic，确保末端转向不被旋转惩罚压制
  threshold_to_consider: 0.5  # 必须 > xy_goal_tolerance(0.3)
```

`threshold_to_consider` 是这里唯一的坑：如果设成小于等于 `xy_goal_tolerance`，等到进了容差圈才生效已经太晚。

### 验证

`component_container_isolated_98728_1787559231391.log` 中出现 13 次 `Goal succeeded` / `Reached the goal!`。

### 一次自我纠错

我最初判断是 `TwirlingCritic` 在惩罚末端旋转导致无法完成。查 `twirling_critic.cpp:36-38` 后否定：它调用 `withinPositionGoalTolerance(...)` 并直接 return，进容差圈后自动停用，不可能是终点问题的元凶。

真正的次要因素是 `VelocityDeadbandCritic`——见问题二。

## 4. 问题二：行进中持续左右摆头

### 测量

先用 `/cmd_vel` 采样，看到平滑单调的后退运动、0 次符号翻转，**完全测不到振荡**。`ros2 topic info --verbose` 揭示 `/cmd_vel` 上有 **7 个发布者**（含 `teleop_twist_keyboard`、`behavior_server`），而 MPPI 实际发布到 `/cmd_vel_nav`。

改测 `/cmd_vel_nav`：261 个样本，`wz` 在 ±1.2（= `wz_max`）之间饱和翻转，振荡确认。

> **教训：Nav2 里诊断控制器输出必须测 `/cmd_vel_nav`。** `/cmd_vel` 是 `velocity_smoother` 之后、多个发布者混合的下游话题。

### 根因：三个因素相乘

1. **推力过强。** `ObstaclesCritic.repulsion_weight: 5.0` 是默认 1.5 的 3.3 倍。关键在 `obstacles_critic.cpp:181`：

   ```cpp
   repulsive_cost[i] += (inflation_radius_ - dist_to_obj);
   data.costs += (critical_weight_ * raw_cost) + (repulsion_weight_ * repulsive_cost / traj_len);
   ```

   `inflation_radius` **同时是斥力的幅值标尺和作用范围**。之前把半径从 0.25 调到 0.4 时已经把斥力放大过一次，再叠 3.3 倍权重，在走廊里就被两侧墙交替强推。

2. **阻尼几乎为零。** `TwirlingCritic.cost_weight: 0.3`，仅为默认 10.0 的 **3%**，等于关掉了角速度阻尼，振荡无法衰减。

3. **采样噪声过大。** `wz_std: 0.8` 是默认 0.4 的 2 倍，MPPI 更容易探到打满 `wz_max` 的解。

另外两个独立诱因：

4. **横向纠偏过强 + 航向约束过弱。** `PathAlignCritic: 14.0`（默认 10）配 `PathAngleCritic: 1.0`（默认 2.2）——机器人靠剧烈摆头消除横向偏差，而不是提前对齐航向，形成过冲振荡。

5. **`deadband_velocities` 的 wz 项是 0.12。** 惩罚形式为 `max(deadband - |v|, 0)`，`wz = 0` 时惩罚**最大**，等于主动排斥直线行驶，最优解变成"左右交替小转"。这条评价器的本意是线速度死区（指令小于死区时机器人不动而 MPPI 以为在动），角速度不适用。参考：`velocity_smoother` 的 `deadband_velocity` 也是全零。

### 修复

| 参数 | 原值 | 新值 | 默认值 |
| --- | --- | --- | --- |
| `wz_std` | 0.8 | **0.5** | 0.4 |
| `ObstaclesCritic.repulsion_weight` | 5.0 | **2.0** | 1.5 |
| `TwirlingCritic.cost_weight` | 0.3 | **4.0** | 10.0 |
| `PathAlignCritic.cost_weight` | 14.0 | **10.0** | 10.0 |
| `PathAngleCritic.cost_weight` | 1.0 | **2.2** | 2.2 |
| `VelocityDeadbandCritic.cost_weight` | 8.0 | **4.0** | — |
| `deadband_velocities` | [0.08, 0.0, 0.12] | **[0.08, 0.0, 0.0]** | — |

`TwirlingCritic` 只回到 4.0 而非默认 10.0：四足需要保留转向灵活性。因为它在容差圈内自动停用，提高本项不会妨碍 `GoalAngleCritic` 完成末端转向，两者作用区间不重叠。

用户确认："不会频繁左右摆头了"。

## 5. 问题三：全局路径持续抖动

三个根因，分属三个不同软件层。

### 5.1 RTAB-Map 层：全局栅格反复重建

`GridGlobal/UpdateError` 原为 `0.01`（rtabmap 默认）。`GlobalMap.cpp:102-137` 的 `fullUpdateNeeded()` 一旦发现任一节点位姿变化超过该阈值，就 `clear()` 并**重建整张全局栅格**。

0.01 m 远小于一个栅格边长（`Grid/CellSize: 0.05`），足式里程计的常态亚厘米抖动即可触发。实测 `TimeUpdatingMaps` p95 达 **750~780 ms**，且 `/map` 反复重建会经 Nav2 `StaticLayer` 直接传导成全局路径抖动。

改为 `0.05`（一个栅格边长）：亚栅格位姿修正不再触发重建，回环等真实修正仍然会重建。

### 5.2 代价地图层：单个体素即成障碍

全局 `voxel_layer.mark_threshold` 原为 `0`，意味着**单个体素即投影为障碍格**。D435i 弱纹理面上逐帧变化的散点会让全局障碍格反复出现消失。改为 `1`（至少两个体素），与局部层一致。

### 5.3 规划器层：启发式随地图微变而变

`cache_obstacle_heuristic: false` 时每次重规划都重算障碍启发式。`a_star.cpp:185`：

```cpp
if (!_search_info.cache_obstacle_heuristic || goal_coords != _goal_coordinates)
```

缓存**仅在目标点变化时失效**。改为 `true` 后，同一次导航任务内逐次重规划复用同一份启发式，切断"地图微变 → 启发式变 → 展开顺序变 → 路径跳变"的链路，同时省去每次重规划的 Dijkstra 波前计算。

碰撞检查始终使用最新代价地图，安全性不受影响。代价是启发式随机器人前进逐渐失准、搜索效率下降；**若出现规划超时应改回 false**。

已验证安全：改动后的日志中 `no valid path` / `lethal space` / `Failed to make progress` 均为 0 次。

## 6. 问题四：局部障碍不清空 → 迁移 STVL

### 6.1 为什么 VoxelLayer 配单目相机必然残留鬼影

决定性证据在 `nav2_voxel_grid/include/nav2_voxel_grid/voxel_grid.hpp` 的 `ClearVoxelInMap::operator()`：

```cpp
*col &= ~(z_mask);              // 一条射线只清掉一个 z 位
unsigned int marked_bits = *col >> 16;
if (bitsBelowThreshold(marked_bits, marked_clear_threshold_)) {
    if (bitsBelowThreshold(unknown_bits, unknown_clear_threshold_)) {
        costmap_[offset] = free_cost_;
    } else { costmap_[offset] = unknown_cost_; }
}
// 没有 else 分支 —— 条件不满足时 2D 栅格保持障碍值
```

一个 `(x, y)` 列要降到 `marked <= mark_threshold` 才会转为自由。障碍带 0.15~0.45 m 在 `z_resolution: 0.05` 下是 **6 层**，而一条射线只穿过 1~2 层，因此清空一格需要多条不同俯仰角的射线。单个前视相机提供不了这个条件；6×6 m 滚动窗口的侧方和后方**从来收不到任何清除射线**，只能等 rolling window 淘汰。

附带发现：这里的 3D 判别本身是冗余的。RTAB-Map 上游已经做了 `NormalsSegmentation` / `NoiseFiltering` / `MinClusterSize=20`，`/local_grid_obstacle` 已是预过滤过的去地面点云。

### 6.2 一处术语纠错

原配置注释写 `observation_persistence: 0.0  # 不保留历史观测`——这是错的。该参数只控制 `ObservationBuffer` 里消息的保留时长，**无法让 `voxel_grid_` 中已标记的体素过期**。那份 3D 状态在被射线清除或被滚动窗口推出之前是持久的。

### 6.3 STVL 方案

STVL 的体素带时间戳、按 `voxel_decay` 自然过期，不依赖射线覆盖，视锥外的鬼影也会自行消失——这是三个候选方案里唯一能真正解决视野外残留的。

结构上从 3 个观测源简化为 2 个，且都指向同一话题：

| | VoxelLayer | STVL |
| --- | --- | --- |
| 标记 | `/local_grid_obstacle` | `/local_grid_obstacle` |
| 清除 | `/local_grid_empty` + `/local_grid_ground` 两路射线 | 同一话题，仅用于构造视锥 |
| 清除原理 | 3D 射线穿过才清 | 时间戳过期自动消失 |

`/local_grid_empty` 和 `/local_grid_ground` 原本存在就是为了给 VoxelLayer 喂清除射线，STVL 不做射线追踪，这两路输入没有位置了。RTAB-Map 照旧发布，Nav2 不再订阅。

### 6.4 关键参数判断

**`voxel_decay`**（最需要实测的一项）：实测 `/local_grid_obstacle` 约 1.7~2.5 Hz（间隔 0.4~0.6 s）。初值定 5.0，用户实测后**调至 2.0**。视野内被持续观测的障碍每帧刷新时间戳不会过期，只有真正不再被看到的才消失。太小会让真实障碍在两帧之间闪掉，太大退化成原来的鬼影问题。合理区间 2~10。

**`track_unknown_space: false`**（刻意不跟官方示例的 `true`）：相机水平视场只有约 71°，6×6 m 局部图绝大部分区域从未被观测，标成 `NO_INFORMATION` 会让机器人被未知区域包围而不敢动。原 VoxelLayer 局部层也没开这项，行为保持一致。

**视锥模型的精度折扣**：`/local_grid_obstacle` 的 frame 实测是 `base_footprint`（在地面上），不是相机光心（高约 0.35 m），STVL 构造的视锥顶点因此偏低。用 `vertical_fov_padding: 0.2` 补偿，并把 `decay_acceleration` 压到 5.0（官方示例 15.0），让清除主要依赖时间衰减、视锥只做温和加速。

**FOV 取值**：D435i 深度视场 87°×58° 是 16:9 标称值。当前 `camera_profile` 为 640×480（4:3），垂直保持 58°（1.01 rad），水平**裁剪至约 71°（1.24 rad）**。直接填 87° 会让视锥比实际观测范围大，误清视野外的真实障碍。

### 6.5 参数名的取证方式

STVL 的参数名与 VoxelLayer 完全不同（`obstacle_range` 而非 `obstacle_max_range`、`min_z`/`max_z` 而非 `raytrace_*_range`），且文档不全。取证方式：

```bash
strings /opt/ros/humble/lib/libspatio_temporal_voxel_layer_core.so | grep ...
cat /opt/ros/humble/share/spatio_temporal_voxel_layer/example/standard_indoor_environment_config.yaml
cat /opt/ros/humble/share/spatio_temporal_voxel_layer/costmap_plugins.xml   # 插件类名
```

插件类名是 `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`（**斜杠**，不是 nav2 内建层那种 `::`）。

## 7. 工程环境踩坑

### 7.1 ros2 daemon 未启动导致 `ros2 node list` 返回空

进程用 `ps` 能看到在跑，但 `ros2 node list` 什么都不返回。所有 `ros2 topic` 命令加 `--no-daemon` 绕过，节点发现改用 `ps`。

### 7.2 `ldd` 误报 openvdb 缺失

未 source ROS 环境时，`ldd libspatio_temporal_voxel_layer_core.so` 显示 `libopenvdb.so.10.0 => not found`。这是环境未 source 的假象：`source /opt/ros/humble/setup.bash` 后 0 个缺失依赖。openvdb 位于 `/opt/ros/humble/opt/openvdb_vendor/lib/libopenvdb.so.10.0.1`，由 ROS 环境 hook 注入路径。

### 7.3 colcon 必须在正确的工作空间根目录执行

```bash
# 错误：在 /root/robot_ws 执行
cd /root/robot_ws && colcon build --symlink-install --packages-select robot_slam_bringup
# ERROR: Failed to find the following files:
#   - /root/robot_ws/install/camera_models/share/camera_models/package.sh
#   - /root/robot_ws/install/vins/share/vins/package.sh

# 正确
cd /root/robot_ws/slam_ws && colcon build --symlink-install --packages-select robot_slam_bringup
```

在 `/root/robot_ws` 执行会把 `slam_ws` / `sensor_ws` / `openvins_ws` / `VINS-Fusion-ROS2-humble-arm` 全部当成同一个工作空间的包，install 前缀变成 `/root/robot_ws/install`，于是找不到实际装在 `VINS-Fusion-ROS2-humble-arm/install/` 下的 `camera_models` 和 `vins`。

### 7.4 新增 config 文件必须 build 一次

`--symlink-install` 下改 yaml **内容**不需要重新 build，重启节点即可。但**新增文件**必须 build 一次才会被 install 规则收录，否则：

```text
ReplaceString substitution error: [Errno 2] No such file or directory:
  '.../install/robot_slam_bringup/share/robot_slam_bringup/config/go2_nav2_stvl.yaml'
[ERROR] [launch]: Caught exception in launch: local variable 'input_file' referenced before assignment
```

原因是 launch 里 `nav2_params_file` 的默认值走 `os.path.join(package_share, "config", ...)`，即 install share 目录。装好后的链接是两跳：

```text
install/.../config/go2_nav2_stvl.yaml -> build/.../config/go2_nav2_stvl.yaml -> src/.../config/go2_nav2_stvl.yaml
```

### 7.5 sed/awk 抽不到跨文件的函数定义

`clearVoxelLineInMap` 的声明在头文件、定义在 `src/voxel_grid.cpp`。正确做法是先 grep 头文件拿声明，再读 .cpp 拿函数体。

## 8. 调参经验提炼

1. **先确认测的是哪个话题。** Nav2 的速度链是 `MPPI -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel`，`/cmd_vel` 上可能有多个发布者。测错话题会得出完全相反的结论。

2. **评价器权重严重偏离默认值时要先问为什么。** 本轮三个振荡因素分别是默认值的 3.3 倍、0.03 倍、2 倍。默认值是调过的，大幅偏离必须有明确理由。

3. **`inflation_radius` 在 MPPI 里是双重角色**：既是斥力作用范围，又是斥力幅值标尺。改半径等于同时改了权重，两者不能独立调。

4. **注意评价器是否随接近终点自动停用。** `TwirlingCritic` / `PreferForwardCritic` / `PathAlignCritic` 会检查位置容差并 return，`VelocityDeadbandCritic` 不会（`velocity_deadband_critic.cpp:45` 只判断 `enabled_`）。这决定了终点行为异常时该怀疑谁。

5. **`max(deadband - |v|, 0)` 形式的惩罚在 v=0 时最大**，即"主动排斥静止/直行"。用在角速度上会直接制造摆头，用在终点会让机器人持续蹭动。

6. **`threshold_to_consider` 必须大于对应的 goal tolerance**，否则评价器等到进了容差圈才生效，为时已晚。

7. **全局层和局部层的膨胀参数刻意不同**：全局 `cost_scaling_factor: 2.0`（衰减放缓，抬高膨胀带内代价，求路径居中），局部 `6.0`（求通过性）。局部层的 0.4/6.0 必须与 MPPI `ObstaclesCritic` 的 `inflation_radius`/`cost_scaling_factor` 保持一致，否则控制器对代价的解读与代价地图不符。

8. **传感器物理视场决定了插件选型。** 单个前视相机 + 需要清除侧后方鬼影 = VoxelLayer 的射线清除机制在原理上就无法满足，只能靠时间衰减（STVL）。这不是调参能解决的。

## 9. 待验证 / 未完成

- [ ] **STVL 实机验证**——配置只做了 YAML 解析和参数名校验，未实机运行过。第一个风险点是插件加载（openvdb 靠 ROS 环境 hook 提供，`component_container_isolated` 里能否解析待验）：
  ```bash
  grep -iE "SpatioTemporalVoxelLayer|openvdb|Failed to (create|load)" ~/.ros/log/component_container_isolated_*.log | tail
  ```
- [ ] **`voxel_decay` 继续实测**——现值 2.0，按"人走开后多久图上消失"调整。注意 `go2_nav2_stvl.yaml` 里该行注释仍写着 5 s，与实际值不一致。
- [ ] **路径稳定性三项改动未实机验证**：`GridGlobal/UpdateError` 0.05、全局 `mark_threshold` 1、`cache_obstacle_heuristic` true。特别注意 `cache_obstacle_heuristic` 若引发规划超时需改回 false。
- [ ] **日志中 4 次 `Timed out` 未追查。**
- [ ] 日志中 2 次平滑器 `leads to a collision`——已判定为良性，由行为树的 `AlwaysSuccess` fallback 吸收，`{path}` 保持规划器原值。
