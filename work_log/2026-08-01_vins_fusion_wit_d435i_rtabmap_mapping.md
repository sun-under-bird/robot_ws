# 2026-08-01 VINS-Fusion + WIT IMU + D435i + RTAB-Map 工作总结

## 1. 工作目标

今天完成的目标是将 D435i 红外双目和外置 WIT IMU 接入 VINS-Fusion，使用 VINS-Fusion 输出的 `/odometry` 作为外部里程计，再由 RTAB-Map 完成建图、视觉回环和可视化。

最终数据链路如下：

```text
D435i 左右红外矫正图像 ─┐
                         ├─> VINS-Fusion ── /odometry ─┐
WIT /imu/data_raw ───────┘                              │
                                                       ├─> RTAB-Map 建图与回环
D435i 左右图像 + CameraInfo ───────────────────────────┘
                                                            │
                                                            ├─> rtabmap_viz
                                                            └─> RTAB-Map 数据库
```

系统职责划分：

- VINS-Fusion：只提供连续局部 VIO 里程计和 `world -> body` TF。
- RTAB-Map：使用外部 `/odometry`，从双目图像独立提取回环特征，完成建图和全局回环优化。
- 不启动 VINS `loop_fusion`，避免两套回环同时修改轨迹。
- 不启动 `rtabmap_odom`，避免产生第二套视觉里程计。

## 2. 新增和调整的文件

### 2.1 VINS-Fusion 参数文件

新增文件：

```text
/home/bird/robot_ws/VINS-Fusion-ROS2-humble-arm/config/realsense_d435i/realsense_stereo_wit_imu_config.yaml
```

主要内容：

- IMU：`/imu/data_raw`
- 左目：`/camera/camera/infra1/image_rect_raw`
- 右目：`/camera/camera/infra2/image_rect_raw`
- 图像尺寸：`640x480`
- 使用 `left.yaml`、`right.yaml` 中的矫正双目内参。
- 根据 `openvins_rtabmap.launch.py` 中已有联合标定外参，计算并写入 `body_T_cam0`、`body_T_cam1`。
- 右目相对左目使用 D435i 工厂基线 `0.0500395522 m`。
- 固定标定外参：`estimate_extrinsic: 0`。
- 固定标定时间偏移：`estimate_td: 0`、`td: 0.013116`。
- 使用 WIT IMU 噪声参数：

```yaml
acc_n: 0.08
gyr_n: 0.01
acc_w: 0.004
gyr_w: 0.001
g_norm: 9.81
```

当前性能参数状态：

```yaml
multiple_thread: 1
max_cnt: 200
min_dist: 25
freq: 10
show_track: 0
flow_back: 1
max_solver_time: 0.04
max_num_iterations: 6
```

说明：`show_track: 0` 和 `max_num_iterations: 6` 已写入当前文件，但修改后遇到了 USB 总线掉线，尚未在传感器健康状态下完成长时间复测。

### 2.2 一体化启动文件

新增文件：

```text
/home/bird/robot_ws/slam_ws/src/robot_slam_bringup/launch/d435i_wit_vins_rtabmap.launch.py
```

启动文件可以统一启动：

- D435i 红外双目
- WIT IMU
- 标定后的 `body -> camera_link` 静态 TF
- VINS-Fusion
- RTAB-Map
- `rtabmap_viz`

关键设计：

- D435i 默认使用 `640x480x15`，避免 30 Hz 给 VINS 特征跟踪造成持续积压。
- 关闭 D435i 彩色、深度、点云、内置陀螺仪和加速度计，只保留红外双目。
- D435i 使用同步双目、固定曝光 `5000` 和增益 `16`。
- WIT IMU 期望频率为 200 Hz。
- VINS 输出 `/odometry`，RTAB-Map 直接订阅该话题。
- RTAB-Map 的 `odom_frame_id` 保持为空，使其进入外部 odom 订阅模式。
- `Mem/UseOdomFeatures=false`，强制 RTAB-Map 从图像重新提取自己的回环词袋特征。
- 默认数据库为 `~/.ros/rtabmap_vins_fusion_mapping.db`。
- 支持 `start_sensors`、`launch_viz`、启动延时、数据库路径、话题重映射等参数。

### 2.3 包依赖

调整文件：

```text
/home/bird/robot_ws/slam_ws/src/robot_slam_bringup/package.xml
```

补充运行依赖：

```xml
<exec_depend>realsense2_camera</exec_depend>
<exec_depend>vins</exec_depend>
```

### 2.4 RTAB-Map 参数复用

启动文件复用：

```text
/home/bird/robot_ws/slam_ws/src/robot_slam_bringup/config/openvins_rtabmap.yaml
```

注意：该文件在工作区中还有其他已有改动，包括 OpenVINS 噪声、ZUPT 和最近帧处理策略；本次新增启动文件只复用其中 `rtabmap` 和 `rtabmap_viz` 的参数，并通过 launch 内参数覆盖外部 odom、双目订阅和 RTAB-Map 回环职责。

## 3. 构建步骤

### 3.1 构建 VINS-Fusion

```bash
source /opt/ros/humble/setup.bash
source /home/bird/rtabmap_humble_ws/install/local_setup.bash
source /home/bird/robot_ws/sensor_ws/install/local_setup.bash

cd /home/bird/robot_ws/VINS-Fusion-ROS2-humble-arm
colcon build --symlink-install --packages-select vins
source install/local_setup.bash
```

构建结果：

- `vins_lib`、`vins_node` 和测试程序构建成功。
- CMakeLists 中使用 Release 构建。
- 新参数文件已安装到 `install/vins/share/vins/config/realsense_d435i/`。

### 3.2 构建 bringup 包

```bash
source /opt/ros/humble/setup.bash
source /home/bird/rtabmap_humble_ws/install/local_setup.bash
source /home/bird/robot_ws/sensor_ws/install/local_setup.bash
source /home/bird/robot_ws/VINS-Fusion-ROS2-humble-arm/install/local_setup.bash

cd /home/bird/robot_ws/slam_ws
colcon build --symlink-install --packages-select robot_slam_bringup
source install/local_setup.bash
```

构建结果：`robot_slam_bringup` 构建和安装成功，新 launch 已进入包共享目录。

## 4. 推荐的完整启动步骤

### 4.1 硬件准备

1. D435i 直接接电脑 USB 3.x 端口。
2. WIT 尽量接另一个物理 USB 端口，不与 D435i 共用无源 Hub。
3. 确认没有另一套 RealSense 或 WIT 驱动正在运行，避免重复节点和重复话题。
4. 创建 VINS 输出目录：

```bash
mkdir -p /home/bird/robot_ws/output
```

### 4.2 加载环境

每个新终端都执行：

```bash
source /opt/ros/humble/setup.bash
source /home/bird/rtabmap_humble_ws/install/local_setup.bash
source /home/bird/robot_ws/sensor_ws/install/local_setup.bash
source /home/bird/robot_ws/VINS-Fusion-ROS2-humble-arm/install/local_setup.bash
source /home/bird/robot_ws/slam_ws/install/local_setup.bash

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROBOT_OUTPUT_DIR=/home/bird/robot_ws/output
```

### 4.3 推荐：由一个 launch 管理全部节点

```bash
ros2 launch robot_slam_bringup d435i_wit_vins_rtabmap.launch.py \
  start_sensors:=true \
  vins_startup_delay:=3.0 \
  rtabmap_startup_delay:=12.0 \
  launch_viz:=true \
  delete_db_on_start:=true
```

启动注意事项：

- 启动后保持机器人静止 5～10 秒，让陀螺仪零偏和 VINS 初始状态稳定。
- `delete_db_on_start:=true` 会删除同路径旧数据库；需要保留旧图时改为 `false` 或换一个 `database_path`。
- 建图时先慢速直行，避免初始化后立即快速原地旋转。
- 正式长时间建图若要降低负载，可使用 `launch_viz:=false`。

### 4.4 复用已经单独启动的传感器

只有在左右图和 WIT IMU 已确认正常时才使用：

```bash
ros2 launch robot_slam_bringup d435i_wit_vins_rtabmap.launch.py \
  start_sensors:=false \
  rtabmap_startup_delay:=12.0 \
  launch_viz:=true
```

不要同时让独立终端和一体化 launch 重复启动 D435i/WIT。

## 5. 启动后的验证步骤

### 5.1 先验证传感器

```bash
ros2 topic hz /camera/camera/infra1/image_rect_raw --window 50
ros2 topic hz /camera/camera/infra2/image_rect_raw --window 50
ros2 topic hz /imu/data_raw --window 200
```

预期：

- 左红外：约 15 Hz
- 右红外：约 15 Hz
- WIT IMU：约 200 Hz

三路中任何一路没有数据，都不要继续判断 RTAB-Map 参数或时间同步。

### 5.2 验证 VINS

```bash
ros2 topic echo /odometry --once
ros2 run tf2_ros tf2_echo world body
```

VINS 日志中应依次看到：

```text
waiting for image and imu...
gyroscope bias initial calibration
Initialization finish!
solver costs: ... [ms]
```

如果始终只有 `waiting for image and imu...`，说明 VINS 没有形成图像和 IMU 测量，不会发布 `/odometry`。

### 5.3 验证 RTAB-Map

```bash
ros2 node info /rtabmap
ros2 topic info -v /odometry
```

RTAB-Map 日志应显示订阅：

```text
/odometry
/camera/camera/infra1/image_rect_raw
/camera/camera/infra2/image_rect_raw
/camera/camera/infra1/camera_info
/camera/camera/infra2/camera_info
```

并持续出现：

```text
rtabmap (1): ...
rtabmap (2): ...
...
```

### 5.4 可视化和离线查看

- 在线使用 `rtabmap_viz`，它不是 RViz2。
- 建图结束后可离线打开数据库：

```bash
rtabmap-databaseViewer /home/bird/.ros/rtabmap_vins_fusion_mapping.db
```

## 6. 已完成的运行验证

### 6.1 成功运行

11:44:51 的运行持续约 95 秒，验证结果：

- VINS 正常完成初始化。
- RTAB-Map 连续执行 84 次更新。
- RTAB-Map 工作内存增长到 68 个节点。
- 没有再次出现 `Did not receive data since 5 seconds`。
- RTAB-Map 数据库正常生成，约 41 MB。
- RTAB-Map 单次处理约 0.1～0.4 秒，低于配置的 0.7 秒限制。

相机降为 15 Hz、固定外参和固定时间偏移后，VINS 求解统计为：

- 平均：32.88 ms
- 最大：77.44 ms
- 超过 40 ms：32.8%
- 超过 66.7 ms（15 Hz 一帧周期）：1.7%

该结果说明链路已经具备连续建图能力，但 VINS 后半段仍存在一定 CPU 峰值。

### 6.2 当前性能修改的复测状态

成功运行时仍使用了较重的显示/迭代配置；此后进一步设置了：

```yaml
show_track: 0
max_num_iterations: 6
```

随后 USB 设备掉线，因此这两项优化的最终耗时收益尚未可靠量化。下次硬件恢复后应再次运行至少 3～5 分钟并统计 `solver costs`。

## 7. 遇到的问题和解决方法

### 7.1 VINS 不能用普通 ROS 2 `Node` 动作启动

现象：使用 `launch_ros.actions.Node` 启动时，ROS 2 会自动向命令追加 `--ros-args`，而当前 VINS 主程序要求参数数量严格等于 2，只接受一个配置文件路径。

解决：启动文件中使用 `ExecuteProcess` 直接执行：

```text
<vins install>/lib/vins/vins_node <config path>
```

经验：移植自 ROS 1 或自定义入口的程序如果严格检查 `argc`，不一定能直接使用 ROS 2 `Node` 动作。

### 7.2 RTAB-Map 没有使用外部 `/odometry`

现象：如果错误设置 `odom_frame_id`，RTAB-Map 会尝试从 TF 获取 odom，而不是订阅外部 odom 话题。

解决：

```yaml
odom_frame_id: ""
subscribe_odom_info: false
```

同时 remap：

```text
odom -> /odometry
```

日志已经确认 `subscribe_odom = true`。

### 7.3 回环职责冲突

目标是使用 RTAB-Map 回环，而不是 VINS 回环。

解决：

- 只启动 `vins_node`，不启动 VINS `loop_fusion`。
- 不启动 `rtabmap_odom`。
- 设置 `Mem/UseOdomFeatures=false`，让 RTAB-Map从图像提取自己的回环特征。

### 7.4 30 Hz 时 VINS 处理积压，RTAB-Map 后续断流

现象：早期一次 30 Hz 运行中：

- VINS 平均求解 59.68 ms。
- 99.7% 的求解超过配置的 40 ms。
- RTAB-Map 前期正常，运行约 31 秒后反复提示五秒未收到同步数据。
- 相机和 WIT 本身仍在发布，说明不是普通掉线。

原因：图像负载过高，VINS 处理和 odom 时间戳逐渐落后，RTAB-Map 的 10 帧近似同步队列无法继续匹配 odom、左右图和 CameraInfo。

解决：

- D435i 从 30 Hz 降到 15 Hz。
- 固定已标定外参和时间偏移，不在线估计。
- 关闭 D435i 内置 IMU，避免无用数据。
- 后续关闭跟踪图输出，并减少最大求解迭代次数。

经验：增大 RTAB-Map 同步队列只能暂时缓冲，不能解决上游持续积压。

### 7.5 在线估计外参和时间偏移带来额外负担

现象：早期配置为：

```yaml
estimate_extrinsic: 1
estimate_td: 1
td: 0.0
```

VINS 日志显示在线时间偏移逐渐估计到约 9～11 ms，同时求解开销较大。

解决：使用联合标定结果固定：

```yaml
estimate_extrinsic: 0
estimate_td: 0
td: 0.013116
```

经验：已有可信联合标定时，正式建图应固定外参和时间偏移；在线估计更适合专门的标定验证过程。

### 7.6 建图开头发糊、出现重影

现象：初始几秒点云可能出现双墙或重影。

原因：VINS 初始化完成后不到一秒，RTAB-Map 就开始保存节点。此时陀螺零偏、速度、重力方向和滑窗状态仍处于早期收敛阶段。自动曝光过长和快速旋转会进一步放大问题。

解决：

- 启动后保持静止 5～10 秒。
- 启动时传入 `rtabmap_startup_delay:=12.0`。
- 使用固定短曝光。
- RTAB-Map 开始后先慢速直行，避免立即快速旋转。

注意：当前 launch 文件默认延时仍是 4 秒，12 秒需要通过启动参数显式传入，后续也可根据实测修改默认值。

### 7.7 `world -> body` TF 启动警告

现象：RTAB-Map 第一帧曾提示无法查询对应时刻的 `world -> body`。

原因：VINS 刚初始化时，第一条 odom 和第一条动态 TF 的到达顺序存在竞争。

处理：该警告只出现一次，后续 RTAB-Map 连续工作；VINS 已持续发布 `world -> body`，当前属于非致命启动警告。延迟 RTAB-Map 启动可以进一步避免。

### 7.8 Ctrl+C 后 VINS 显示 `exit code -6`

现象：launch 在用户按 Ctrl+C 后报告 VINS `exit code -6`。

判断：错误发生在 SIGINT 之后，而不是建图运行期间；RTAB-Map 和可视化都能正常退出。

原因：当前 VINS 退出时仍有线程清理/析构问题。

处理：暂不影响运行结果，但后续应在 VINS 主程序中完善停止标志、线程 `join()` 和 ROS 关闭顺序。

### 7.9 `rtabmap_viz` 重复 logger 警告

现象：

```text
Publisher already registered for provided node name
```

判断：警告后 `rtabmap_viz` 正常读取参数、建立订阅并启动，属于非致命 rosout/logger 警告。

### 7.10 最新一次“没有订阅”的真实原因

现象：RTAB-Map 不建图，反复提示五秒没有收到数据。

实际链路：

```text
RTAB-Map 已建立五路订阅
        ↓
VINS 一直停在 waiting for image and imu
        ↓
VINS 没有发布 /odometry
        ↓
RTAB-Map 无法组成同步数据包
```

该次运行期间 WIT 仍稳定约 199.5 Hz，但 VINS 没有完成初始化。随后 D435i 和 WIT 在同一秒从 USB 总线断开：

- WIT：`/dev/ttyUSB0` 消失。
- RealSense：`The device has been disconnected!`
- 内核：USB 协议错误 `error -71`，随后断开整个下游 Hub。

结论：RTAB-Map 实际已经订阅，不是简单的时间戳问题；如果 USB 断开不是人为拔线，则强烈指向 USB Hub、供电、接口或线材不稳定。D435i 应直连 USB 3.x，WIT 应尽量使用另一个物理端口。

经验：看到 `Did not receive data` 时，应按“发布端硬件 -> 话题频率 -> VINS odom -> RTAB-Map 同步”顺序排查，不能直接把问题归因于时间戳或 RTAB-Map 参数。

## 8. 性能优化建议

当前优先级：

1. 保持 D435i `640x480x15`。
2. 正式建图使用 `launch_viz:=false`，减少 Qt 可视化占用。
3. 保持 `show_track: 0` 和 `max_num_iterations: 6`。
4. 如果 VINS 仍频繁超过 66.7 ms，再依次尝试：

```yaml
max_cnt: 150
min_dist: 30
freq: 8
```

5. 若 RTAB-Map 本身成为瓶颈，再把：

```yaml
Kp/MaxFeatures: "500"
Vis/MaxFeatures: "600"
```

不要优先通过增大同步队列掩盖上游处理积压；每次只调整一组参数，并保留至少 3～5 分钟日志用于比较。

## 9. 可积累的经验

1. “已经订阅”和“同步回调收到完整数据”是两个不同状态。订阅列表正常并不代表五路消息可以配成一组。
2. 外部 odom 模式下，必须先确认 `/odometry` 是否存在，再检查 RTAB-Map。
3. VINS 的 `waiting for image and imu` 是上游测量未形成，RTAB-Map 参数无法解决。
4. 时间戳问题必须用各话题 `header.stamp` 和频率证明，不能仅凭 RTAB-Map 的通用警告判断。
5. 视觉惯性系统更看重持续实时性，而不是单帧偶尔很快；平均耗时小于周期但存在大量长尾，也可能造成队列积压。
6. D435i 和串口 IMU 共用 Hub 时，USB 供电或协议错误可能同时影响两套传感器；进程仍存在不代表硬件仍在发布。
7. 对已有可靠标定的正式运行配置，应固定外参和时间偏移；在线标定应作为独立实验。
8. 建图开始时间应晚于 VIO 初始化时间，必要时加入就绪检测或足够的启动延时。
9. RTAB-Map 数据库是比 RViz 更完整的离线分析入口，可使用 `rtabmap-databaseViewer` 检查节点、回环、约束和地图。

## 10. 后续工作

1. 恢复并分离 D435i/WIT 的 USB 连接，确认不存在 Hub 供电和 `error -71`。
2. 使用当前 `show_track: 0`、`max_num_iterations: 6` 配置完成 3～5 分钟复测。
3. 统计新的 VINS 平均、最大和 P95 求解耗时。
4. 做一次明确的闭环路线，确认 RTAB-Map 日志产生回环约束，而不是只观察工作内存节点增长。
5. 检查回环后的地图重影是否被图优化消除。
6. 后续使用 Allan 方差实测结果替换当前 WIT IMU 噪声参数。
7. 有需要时修复 VINS Ctrl+C 后线程未干净退出的问题。

