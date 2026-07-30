# robot_ws

用于把传感器驱动、OpenVINS、VINS-Fusion、建图和导航代码迁移到 RK3588 的总仓库。
ROS 2 包按职责拆分：传感器统一放在 `sensor_ws`，建图导航统一放在 `slam_ws`。

## 目录

```text
robot_ws/
├── sensor_ws/                  # 相机、IMU 驱动和传感器数据预处理
├── slam_ws/                    # 建图、定位、状态估计和 Nav2 入口
├── openvins_ws/
├── VINS-Fusion-ROS2-humble-arm/
├── tools/                      # 离线分析和硬件测试工具
└── scripts/                    # 环境、构建和加载脚本
```

本机工作总结统一归档在仓库外的 `/home/bird/work_log`，不再分散保存在各工程
目录中。

以下内容未复制，也不应提交到 Git：

- `build/`、`install/`、`log/` 和缓存；
- rosbag、运行输出、测试结果和 `datasets/`；
- 各子工程的 `.git/`，根目录将作为唯一 Git 仓库；
- `openvins_ws/src/open_vins/ov_data/`，该目录约 377 MB，仅包含评测 ground-truth 数据，不参与实时运行。

## 来源快照

- 原 `camera` 工程：`5e36757e0803813a8c9703d647413bdaadff5513`
- `open_vins`: `69488123ed9362dd44b6f28e7f4680abbff1442b`
- `VINS-Fusion-ROS2-humble-arm`: `ee54c07d3e33ea5ac02816f373fbd322e11b8fa4`
- 原 `imu_ws` 工程：原目录不是 Git 仓库，按当前文件复制。

复制的是当时的工作树，因此也包含原 `camera` 和 VINS-Fusion 中尚未提交的源码、配置及文档修改。

## RK3588 环境

建议使用 Ubuntu 22.04 aarch64 和 ROS 2 Humble，与当前软件栈保持一致。克隆后先安装系统依赖：

```bash
cd ~/robot_ws
source scripts/setup_robot_env.sh
source /opt/ros/humble/setup.bash
sudo rosdep init  # 仅首次使用 rosdep 时执行
rosdep update
rosdep install --from-paths . --ignore-src -r -y --rosdistro "${ROS_DISTRO}"
```

`setup_robot_env.sh` 自动设置：

- `ROBOT_WS_ROOT`：当前仓库根目录；
- `ROBOT_OUTPUT_DIR`：默认 `${ROBOT_WS_ROOT}/output`；
- `ROS_DISTRO`：默认 `humble`。
- `ROBOT_RTABMAP_WS`：若仓库同级存在 `rtabmap_humble_ws`，自动指向该目录。

需要把运行结果写到外接 SSD 时，在 source 前覆盖输出目录：

```bash
export ROBOT_OUTPUT_DIR=/mnt/robot_data/output
source ~/robot_ws/scripts/setup_robot_env.sh
```

然后逐个构建，便于定位 ARM 平台的依赖或编译问题：

```bash
cd ~/robot_ws
source scripts/setup_robot_env.sh
./scripts/build_all.sh
```

构建顺序固定为 `sensor_ws → openvins_ws → VINS-Fusion → slam_ws`。构建完成后，
可以一次加载全部环境：

```bash
source ~/robot_ws/scripts/source_all.sh
```

RTAB-Map 和 Nav2 是 `slam_ws` 的第三方依赖，不复制进本仓库。Nav2 默认使用系统
ROS 2 安装；若使用自编译 RTAB-Map，可在加载环境前显式指定：

```bash
export ROBOT_RTABMAP_WS=/path/to/rtabmap_humble_ws
source ~/robot_ws/scripts/source_all.sh
```

也可以单独构建，例如：

```bash
source ~/robot_ws/scripts/setup_robot_env.sh
cd "${ROBOT_WS_ROOT}/openvins_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential
```

只构建传感器或建图导航工作空间时，按以下顺序加载依赖：

```bash
source /opt/ros/humble/setup.bash
cd "${ROBOT_WS_ROOT}/sensor_ws"
colcon build --symlink-install --executor sequential
source install/setup.bash

cd "${ROBOT_WS_ROOT}/slam_ws"
colcon build --symlink-install --executor sequential
```

## 路径约定

运行时代码不依赖开发机用户名：ROS 包内资源通过 ament 包索引定位，VINS 输出使用 `ROBOT_OUTPUT_DIR`，外部 bag 和数据集通过启动参数传入。可以用以下命令检查是否重新引入了用户目录：

```bash
rg -n '/home/(yahboom|elephant|patrick|tony-ws1|tong|dji)|/media/(patrick|tony-ws1)' . \
  -g '!**/build/**' -g '!**/install/**' -g '!**/log/**'
```

真实相机测试还需要在 RK3588 上重新安装 RealSense/USB udev 规则；udev 规则属于系统配置，不应只依赖此代码仓库。

## 上传 GitHub

建议使用私有仓库，因为其中包含设备标定参数。确认没有密钥、设备序列号或不希望公开的配置后执行：

```bash
cd ~/robot_ws
source scripts/setup_robot_env.sh
git init
git add .
git commit -m "Import robot ROS 2 workspaces"
git branch -M main
git remote add origin <你的 GitHub 仓库地址>
git push -u origin main
```

不要使用 Git LFS 保存 `build/`、数据集或 rosbag；它们应通过独立下载或局域网传输。
