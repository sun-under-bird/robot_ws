# 2026-07-31 Kalibr 相机-IMU 外参转换与写入记录

## 1. 工作目标和结果

将 `/home/bird/kalibr_data` 中最新的相机-IMU 联合标定结果转换为 ROS 2
`static_transform_publisher` 使用的平移和四元数，并更新：

```text
/home/bird/robot_ws/slam_ws/src/robot_slam_bringup/launch/openvins_rtabmap.launch.py
```

本次按修改时间选中的标定文件是：

```text
/home/bird/kalibr_data/cam_imu_repeat_01-camchain-imucam.yaml
/home/bird/kalibr_data/cam_imu_repeat_01-results-imucam.txt
/home/bird/kalibr_data/cam_imu_repeat_01-report-imucam.pdf
```

它们生成于 2026-07-31 11:58，是目录中最新的一组完整联合标定结果。

最终写入 launch 的 `camera_link -> imu_link` 变换为：

```text
translation xyz [m]:
[-0.037283913338684735,
 -0.021068476495775222,
  0.004906068911756066]

quaternion xyzw:
[ 0.7127580390029497,
 -0.00127832915681575,
 -0.7013739231475679,
  0.00699740236273522]
```

对应的静态 TF 参数是：

```python
arguments=[
    "--x", "-0.037283913338684735",
    "--y", "-0.021068476495775222",
    "--z", "0.004906068911756066",
    "--qx", "0.7127580390029497",
    "--qy", "-0.00127832915681575",
    "--qz", "-0.7013739231475679",
    "--qw", "0.00699740236273522",
    "--frame-id", "camera_link",
    "--child-frame-id", "imu_link",
]
```

## 2. 最新标定结果摘要

Kalibr 的 `cam0` 结果为：

```text
T_cam_imu（imu0 到 cam0）:
[[ 0.011637869860011894,  0.9998988044694812,    0.008181736100775943,  0.021068476495775222],
 [ 0.9998019141740071,   -0.011768083043935087,  0.016051312577527177, -0.004906068911756066],
 [ 0.01614597160631287,   0.007993312327962736, -0.9998376941078562,   -0.037283913338684735],
 [ 0.0,                    0.0,                   0.0,                    1.0]]
```

其他重要结果：

- `cam0` 平均重投影误差：`0.2244568939 px`
- `cam1` 平均重投影误差：`0.2424592851 px`
- 双目基线：`0.050039552364669865 m`
- `cam0` 时间偏移：`0.01311671095948102 s`
- 时间关系：`t_imu = t_cam + 0.01311671095948102 s`

`cam1` 的时间偏移是 `0.013102572586538864 s`。OpenVINS 当前只使用第一
个相机的相机-IMU时间偏移，因此应以 `cam0` 的值为准。

注意：时间偏移不属于空间 TF，不能放进 `static_transform_publisher`。
当前配置文件中的 `OdomOpenVINS/CalibCamTimeoffset` 为 `true`，OpenVINS
会在运行时继续估计时间偏移。本次只更新了 launch 中实际承载空间标定结果的
静态 TF，没有把未被节点消费的“注释参数”伪装成有效配置。

相机内参、零畸变和双目基线已经由标定时使用的 D435i 矫正图像及驱动
`CameraInfo` 提供；IMU 噪声是联合标定的输入先验，不是这次联合标定新估计
出来的结果。因此本次没有把这些值重复塞进静态 TF，也没有覆盖其他配置文件。

## 3. 为什么不能直接复制 `T_cam_imu`

Kalibr 文件中的：

```text
T_cam_imu = T_cam0_imu
```

表示把 IMU 坐标中的点变换到左目光学坐标系：

```text
p_cam0 = T_cam0_imu * p_imu
```

因此，如果 TF 的父坐标系直接使用 `camera_infra1_optical_frame`，那么它正好
就是“父相机、子 IMU”的变换，不需要求逆。

目标 launch 使用的父坐标系是 ROS 相机机体坐标系 `camera_link`，不是左目
光学坐标系。两者的轴定义不同：

```text
camera_link: x 向前，y 向左，z 向上
cam0 optical: x 向右，y 向下，z 向前
```

所以光学坐标到 `camera_link` 坐标的关系是：

```text
[x_link, y_link, z_link] = [z_cam0, -x_cam0, -y_cam0]
```

对应矩阵为：

```text
T_camera_link_cam0 =
[[ 0,  0,  1,  0],
 [-1,  0,  0,  0],
 [ 0, -1,  0,  0],
 [ 0,  0,  0,  1]]
```

最终需要做左乘：

```text
T_camera_link_imu =
    T_camera_link_cam0 * T_cam0_imu
```

合成结果是：

```text
T_camera_link_imu =
[[ 0.01614597160631287,   0.007993312327962736, -0.9998376941078562,   -0.037283913338684735],
 [-0.011637869860011894, -0.9998988044694812,   -0.008181736100775943, -0.021068476495775222],
 [-0.9998019141740071,    0.011768083043935087, -0.016051312577527177,  0.004906068911756066],
 [ 0.0,                    0.0,                    0.0,                    1.0]]
```

这里沿用了当前 D435i TF 中 `camera_link` 与左红外光学坐标系光心重合、只做
ROS 相机轴到光学轴旋转的约定。如果以后换相机或修改 RealSense 的 frame
配置，应先运行：

```bash
ros2 run tf2_ros tf2_echo camera_link camera_infra1_optical_frame
```

如果输出的平移不为零，必须把实际的完整 4×4 变换代入
`T_camera_link_cam0`，不能继续使用上面的纯旋转矩阵。

## 4. 以后自己转换的可复现步骤

### 4.1 找出最新的联合标定 YAML

```bash
find /home/bird/kalibr_data -maxdepth 1 \
  -type f -name '*-camchain-imucam.yaml' \
  -printf '%T@ %TY-%Tm-%Td %TH:%TM:%TS %p\n' \
  | sort -nr \
  | head
```

不要只看文件名中的日期，也不要从 PDF 中复制已经被截断的小数。应从最新
`*-camchain-imucam.yaml` 读取全精度矩阵，再用同组
`*-results-imucam.txt` 检查残差和矩阵方向。

### 4.2 自动读取 YAML 并转换

下面脚本仅依赖 ROS 2 Humble 环境通常已有的 PyYAML，不依赖 SciPy。保存成
`kalibr_to_tf.py` 后，把 YAML 路径作为第一个参数传入即可。

```python
#!/usr/bin/env python3
"""把 Kalibr cam0 的 T_cam_imu 转为 camera_link -> imu_link 静态 TF。"""

import math
import sys
from pathlib import Path

import yaml


T_CAMERA_LINK_CAM0 = [
    [0.0, 0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


def matrix_multiply(left, right):
    """计算两个二维矩阵的乘积。"""
    # 这里必须保持左乘顺序，交换顺序会得到不同坐标系的变换。
    return [
        [
            sum(left[row][index] * right[index][column]
                for index in range(len(right)))
            for column in range(len(right[0]))
        ]
        for row in range(len(left))
    ]


def rotation_to_quaternion(rotation):
    """把 3×3 旋转矩阵转换为 ROS 顺序的单位四元数 xyzw。"""
    # 按最大对角元素分支，接近 180 度旋转时比单独使用 trace 更稳定。
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qx = (rotation[2][1] - rotation[1][2]) / scale
        qy = (rotation[0][2] - rotation[2][0]) / scale
        qz = (rotation[1][0] - rotation[0][1]) / scale
        qw = 0.25 * scale
    elif rotation[0][0] > rotation[1][1] and \
            rotation[0][0] > rotation[2][2]:
        scale = math.sqrt(
            1.0 + rotation[0][0] -
            rotation[1][1] - rotation[2][2]) * 2.0
        qx = 0.25 * scale
        qy = (rotation[0][1] + rotation[1][0]) / scale
        qz = (rotation[0][2] + rotation[2][0]) / scale
        qw = (rotation[2][1] - rotation[1][2]) / scale
    elif rotation[1][1] > rotation[2][2]:
        scale = math.sqrt(
            1.0 + rotation[1][1] -
            rotation[0][0] - rotation[2][2]) * 2.0
        qx = (rotation[0][1] + rotation[1][0]) / scale
        qy = 0.25 * scale
        qz = (rotation[1][2] + rotation[2][1]) / scale
        qw = (rotation[0][2] - rotation[2][0]) / scale
    else:
        scale = math.sqrt(
            1.0 + rotation[2][2] -
            rotation[0][0] - rotation[1][1]) * 2.0
        qx = (rotation[0][2] + rotation[2][0]) / scale
        qy = (rotation[1][2] + rotation[2][1]) / scale
        qz = 0.25 * scale
        qw = (rotation[1][0] - rotation[0][1]) / scale

    quaternion = [qx, qy, qz, qw]
    norm = math.sqrt(sum(value * value for value in quaternion))
    quaternion = [value / norm for value in quaternion]

    # q 和 -q 表示同一旋转，固定 qw 非负便于不同批次结果做文本比较。
    if quaternion[3] < 0.0:
        quaternion = [-value for value in quaternion]
    return quaternion


def main():
    """读取 Kalibr YAML，合成坐标变换并打印 launch 所需数值。"""
    if len(sys.argv) != 2:
        raise SystemExit(
            f"用法: {Path(sys.argv[0]).name} <camchain-imucam.yaml>")

    yaml_path = Path(sys.argv[1]).expanduser().resolve()
    with yaml_path.open("r", encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)

    # Kalibr 的 T_cam_imu 已经是 IMU 子坐标系在相机父坐标系中的位姿。
    t_cam0_imu = calibration["cam0"]["T_cam_imu"]
    t_camera_link_imu = matrix_multiply(
        T_CAMERA_LINK_CAM0, t_cam0_imu)

    translation = [t_camera_link_imu[index][3] for index in range(3)]
    rotation = [row[:3] for row in t_camera_link_imu[:3]]
    quaternion = rotation_to_quaternion(rotation)
    time_shift = calibration["cam0"]["timeshift_cam_imu"]

    print("translation xyz [m]:", translation)
    print("quaternion xyzw:", quaternion)
    print("time relation:")
    print(f"t_imu = t_cam + {time_shift} s")


if __name__ == "__main__":
    main()
```

运行：

```bash
python3 kalibr_to_tf.py \
  /home/bird/kalibr_data/cam_imu_repeat_01-camchain-imucam.yaml
```

期望输出：

```text
translation xyz [m]: [-0.037283913338684735, -0.021068476495775222, 0.004906068911756066]
quaternion xyzw: [0.7127580390029497, -0.0012783291568157524, -0.7013739231475679, 0.006997402362735216]
time relation:
t_imu = t_cam + 0.01311671095948102 s
```

脚本输出与 launch 中保留到约 16 位有效数字的四元数只有浮点末位舍入差异，
表示的是同一个旋转。

### 4.3 写入 launch 时的检查清单

1. `--frame-id` 是父坐标系，本项目中为 `camera_link`。
2. `--child-frame-id` 是 IMU 子坐标系，本项目中为 `imu_link`。
3. 四元数顺序必须是 `qx qy qz qw`，不能写成常见数学库的 `wxyz`。
4. 平移单位是米，不能误写成毫米。
5. 不要把 `T_ic` 当成 `T_ci`，也不要在已经正确的 `T_cam_imu` 上再求逆。
6. `q` 与 `-q` 表示同一个旋转，比较结果时不能只做字符串比较。
7. 时间偏移不能写入 TF；应由时间戳转发节点或 OpenVINS 配置处理。
8. 如果 RealSense 已经发布左右相机 TF，不要再给同一个相机 child frame
   发布第二个父坐标系，否则 TF 树会冲突。

## 5. 验证方法

### 5.1 Python 语法检查

使用 `ast.parse` 不会在源码目录生成 `__pycache__`：

```bash
cd /home/bird/robot_ws
python3 - <<'PY'
import ast
from pathlib import Path

path = Path(
    "slam_ws/src/robot_slam_bringup/launch/openvins_rtabmap.launch.py")
ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
print("AST syntax: OK")
PY
```

### 5.2 ROS 2 launch 参数解析

```bash
cd /home/bird/robot_ws
mkdir -p /tmp/robot_ws_ros_log
source /opt/ros/humble/setup.bash
source slam_ws/install/setup.bash
ROS_LOG_DIR=/tmp/robot_ws_ros_log \
  ros2 launch robot_slam_bringup openvins_rtabmap.launch.py --show-args
```

本次检查已通过，并显示：

```text
publish_camera_imu_tf:
    是否发布 2026-07-31 标定的 camera_link 到外置 WIT imu_link 静态 TF。
    (default: 'true')
```

### 5.3 运行时 TF 检查

启动传感器和 SLAM 后运行：

```bash
ros2 run tf2_ros tf2_echo camera_link imu_link
```

确认输出平移、旋转与本文结果一致，并检查 TF 树中 `imu_link` 只有一个父节点。

还应观察 OpenVINS 启动日志，确认：

- 能找到 `camera_link`、左右相机光学坐标系和 `imu_link` 之间的 TF；
- IMU 与双目图像持续输入，没有长时间同步超时；
- 初始化后时间偏移估计没有快速发散；
- 实际运动方向与里程计坐标方向一致。

## 6. 工作中遇到的问题及解决方法

### 6.1 `kalibr_data` 不在工作区根目录

最初按相对路径 `kalibr_data` 查找时提示目录不存在。实际目录是：

```text
/home/bird/kalibr_data
```

解决方法是先在允许读取的用户目录中定位：

```bash
find /home/bird -maxdepth 4 -type d -name kalibr_data
```

经验：用户说“我的某目录”时，不应默认它一定在当前仓库根目录；先定位再按
修改时间挑选结果。

### 6.2 目标 launch 已经存在未提交修改

目标文件在本次工作前已经包含 2026-07-30 外参及 topic 等未提交修改。

解决方法是只替换标定日期、节点名和平移/四元数，保留其他已有修改，不执行
`git checkout`、`git reset` 等覆盖操作。

经验：更新标定值前先查看 `git status` 和 `git diff`，把用户改动与本次改动
分开处理。

### 6.3 SciPy 与 NumPy ABI 不兼容

尝试使用 `scipy.spatial.transform.Rotation` 时出现：

```text
ValueError: numpy.dtype size changed, may indicate binary incompatibility
```

当前 SciPy 要求 NumPy `<1.25`，环境中却是 NumPy `2.2.6`。

解决方法是使用本文脚本中的标准旋转矩阵转四元数公式，并通过以下数值性质
验证：

```text
det(R) = 0.9999999999999988
||R^T R - I|| = 1.4770610232134426e-15
||q|| = 0.9999999999999999
四元数回算矩阵误差 = 5.495679536980972e-16
```

经验：这种简单坐标变换不应强依赖大型科学计算库；保留一个无 SciPy 的稳定
转换方法，更适合机器人部署环境。

### 6.4 ROS 默认日志目录不可写

受限环境中运行 `ros2 launch ... --show-args` 时，ROS 尝试写
`/home/bird/.ros/log`，导致只读文件系统错误。

解决方法是将当前检查的日志目录指向可写的 `/tmp`：

```bash
mkdir -p /tmp/robot_ws_ros_log
ROS_LOG_DIR=/tmp/robot_ws_ros_log ros2 launch ...
```

这次重试后 launch 参数解析成功。

## 7. 可积累的经验

- 始终从 `*-camchain-imucam.yaml` 读取全精度值，PDF 和终端报告主要用于检查
  质量，不能作为高精度复制来源。
- `T_a_b` 的可靠理解方法是写出等式 `p_a = T_a_b * p_b`，不要只凭变量名
  猜“谁到谁”。
- ROS 静态 TF 表达“子坐标系在父坐标系中的位姿”。父为相机、子为 IMU 时，
  Kalibr 的 `T_cam_imu` 本身方向正确；是否需要额外合成取决于父相机 frame
  是光学坐标系还是 `camera_link`。
- 坐标系合成必须保持矩阵乘法顺序。要从 `camera_link` 走到 IMU，应写
  `T_camera_link_cam0 * T_cam0_imu`。
- 空间外参、时间偏移、相机内参、IMU 噪声是四类不同参数，不能因为它们都在
  Kalibr 输出附近就全部塞进静态 TF。
- 更新外参后至少检查旋转矩阵正交性、行列式、四元数模长和回算误差，再做
  ROS launch 解析与运行时 TF 检查。
