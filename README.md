# robot_ws

用于把相机、IMU、OpenVINS 和 VINS-Fusion 代码迁移到 RK3588 的总仓库。
四个子工程保留原目录结构，避免改变已经验证过的配置相对路径。

## 目录

```text
robot_ws/
├── camera/
├── imu_ws/
├── openvins_ws/
├── VINS-Fusion-ROS2-humble-arm/
└── scripts/
    ├── setup_robot_env.sh
    └── build_all.sh
```

以下内容未复制，也不应提交到 Git：

- `build/`、`install/`、`log/` 和缓存；
- rosbag、运行输出、测试结果和 `datasets/`；
- 各子工程的 `.git/`，根目录将作为唯一 Git 仓库；
- `openvins_ws/src/open_vins/ov_data/`，该目录约 377 MB，仅包含评测 ground-truth 数据，不参与实时运行。

## 来源快照

- `camera`: `5e36757e0803813a8c9703d647413bdaadff5513`
- `open_vins`: `69488123ed9362dd44b6f28e7f4680abbff1442b`
- `VINS-Fusion-ROS2-humble-arm`: `ee54c07d3e33ea5ac02816f373fbd322e11b8fa4`
- `imu_ws`: 原目录不是 Git 仓库，按当前文件复制。

复制的是当前工作树，因此也包含 `camera` 和 VINS-Fusion 中尚未提交的源码、配置及文档修改。

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

也可以单独构建，例如：

```bash
source ~/robot_ws/scripts/setup_robot_env.sh
cd "${ROBOT_WS_ROOT}/openvins_ws"
source /opt/ros/humble/setup.bash
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
