#!/usr/bin/env python3
"""分析 ROS 2 bag 中的 IMU 冲击、采样质量和纯惯导积分轨迹。"""

import argparse
import math
import warnings
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
warnings.filterwarnings("ignore", message="Unable to import Axes3D.*")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


STANDARD_GRAVITY = 9.80665


def parse_args():
    """解析 bag 路径、IMU topic 和输出文件参数。"""
    parser = argparse.ArgumentParser(
        description="读取 ROS 2 bag，统计 IMU 冲击并绘制纯惯导积分轨迹。"
    )
    parser.add_argument("bag", type=Path, help="ROS 2 bag 目录")
    parser.add_argument(
        "--topic",
        default="",
        help="IMU topic；不填写时自动选择 bag 中唯一的 sensor_msgs/msg/Imu",
    )
    parser.add_argument(
        "--initial-seconds",
        type=float,
        default=1.0,
        help="用于估计初始重力方向和陀螺仪零偏的静止时间，默认 1 秒",
    )
    parser.add_argument(
        "--shock-g",
        type=float,
        default=2.0,
        help="冲击事件阈值，单位 g，默认 2.0 g",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="输出图片路径；默认保存到 BAG/imu_analysis.png",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="输出 CSV 路径；默认保存到 BAG/imu_samples.csv",
    )
    parser.add_argument(
        "--odom-topic",
        default="/odom",
        help="需要与冲击对齐的 nav_msgs/msg/Odometry topic，默认 /odom",
    )
    parser.add_argument(
        "--odom-info-topic",
        default="/odom_info",
        help="可选的 rtabmap_msgs/msg/OdomInfo topic，默认 /odom_info",
    )
    parser.add_argument(
        "--vio-output",
        type=Path,
        default=None,
        help="IMU、里程计和视觉状态对齐图；默认保存到 BAG/vio_imu_timeline.png",
    )
    return parser.parse_args()


def load_bag_metadata(bag_path):
    """读取 bag 元数据，获得存储类型和 topic 类型。"""
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"找不到 {metadata_path}")
    with metadata_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)["rosbag2_bagfile_information"]

    topic_types = {}
    for item in metadata["topics_with_message_count"]:
        topic = item["topic_metadata"]
        topic_types[topic["name"]] = topic["type"]
    return metadata["storage_identifier"], topic_types


def select_imu_topic(topic_types, requested_topic):
    """选择用户指定的 IMU topic，或自动选择唯一的 IMU topic。"""
    if requested_topic:
        if topic_types.get(requested_topic) != "sensor_msgs/msg/Imu":
            raise ValueError(f"{requested_topic} 不存在或不是 sensor_msgs/msg/Imu")
        return requested_topic

    imu_topics = [
        name
        for name, msg_type in topic_types.items()
        if msg_type == "sensor_msgs/msg/Imu"
    ]
    if len(imu_topics) != 1:
        raise ValueError(
            f"检测到 {len(imu_topics)} 个 IMU topic，请使用 --topic 指定：{imu_topics}"
        )
    return imu_topics[0]


def read_imu_samples(bag_path, storage_id, topic_name):
    """从 bag 读取 IMU 时间戳、加速度和角速度。"""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path), storage_id=storage_id
    )
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    msg_type = get_message("sensor_msgs/msg/Imu")
    stamps = []
    accelerations = []
    angular_velocities = []

    while reader.has_next():
        topic, serialized, bag_stamp = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(serialized, msg_type)

        # 优先使用传感器时间戳；驱动没有填写时退回 bag 接收时间。
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            stamp = bag_stamp * 1e-9
        stamps.append(stamp)
        accelerations.append(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ]
        )
        angular_velocities.append(
            [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ]
        )

    if len(stamps) < 3:
        raise RuntimeError(f"{topic_name} 中只有 {len(stamps)} 条有效 IMU 数据")

    stamps = np.asarray(stamps, dtype=np.float64)
    accelerations = np.asarray(accelerations, dtype=np.float64)
    angular_velocities = np.asarray(angular_velocities, dtype=np.float64)

    # 防止异常 bag 中的重复或倒序时间戳破坏积分。
    valid = np.concatenate(([True], np.diff(stamps) > 0.0))
    return stamps[valid], accelerations[valid], angular_velocities[valid]


def read_vio_samples(bag_path, storage_id, odom_topic, odom_info_topic):
    """读取里程计轨迹、协方差以及可选的 RTAB-Map 视觉状态。"""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path), storage_id=storage_id
    )
    reader.open(storage_options, rosbag2_py.ConverterOptions("", ""))

    odom_type = get_message("nav_msgs/msg/Odometry")
    odom_info_type = get_message("rtabmap_msgs/msg/OdomInfo")
    odom_stamps = []
    positions = []
    velocities = []
    position_stds = []
    info_stamps = []
    features = []
    local_map_sizes = []
    lost_flags = []
    estimation_times = []
    intervals = []

    while reader.has_next():
        topic, serialized, bag_stamp = reader.read_next()
        if topic == odom_topic:
            msg = deserialize_message(serialized, odom_type)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp <= 0.0:
                stamp = bag_stamp * 1e-9
            odom_stamps.append(stamp)
            positions.append(
                [
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                ]
            )
            velocities.append(
                [
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ]
            )
            # 只比较平移 x/y/z 的最大标准差，便于发现纯惯导传播阶段。
            covariance_diagonal = [
                msg.pose.covariance[0],
                msg.pose.covariance[7],
                msg.pose.covariance[14],
            ]
            position_stds.append(
                math.sqrt(max(0.0, max(covariance_diagonal)))
            )
        elif topic == odom_info_topic:
            msg = deserialize_message(serialized, odom_info_type)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp <= 0.0:
                stamp = bag_stamp * 1e-9
            info_stamps.append(stamp)
            features.append(msg.features)
            local_map_sizes.append(msg.local_map_size)
            lost_flags.append(msg.lost)
            estimation_times.append(msg.time_estimation)
            intervals.append(msg.interval)

    if len(odom_stamps) < 2:
        raise RuntimeError(f"{odom_topic} 中只有 {len(odom_stamps)} 条里程计数据")

    return {
        "odom_stamps": np.asarray(odom_stamps, dtype=np.float64),
        "positions": np.asarray(positions, dtype=np.float64),
        "velocities": np.asarray(velocities, dtype=np.float64),
        "position_stds": np.asarray(position_stds, dtype=np.float64),
        "info_stamps": np.asarray(info_stamps, dtype=np.float64),
        "features": np.asarray(features, dtype=np.int32),
        "local_map_sizes": np.asarray(local_map_sizes, dtype=np.int32),
        "lost_flags": np.asarray(lost_flags, dtype=bool),
        "estimation_times": np.asarray(estimation_times, dtype=np.float64),
        "intervals": np.asarray(intervals, dtype=np.float64),
    }


def normalize_quaternion(quaternion):
    """归一化四元数，避免长时间积分后的数值误差。"""
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return quaternion / norm


def quaternion_multiply(left, right):
    """计算两个 wxyz 四元数的乘积。"""
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ]
    )


def quaternion_from_two_vectors(source, target):
    """计算把 source 方向旋转到 target 方向的四元数。"""
    source = source / np.linalg.norm(source)
    target = target / np.linalg.norm(target)
    dot = float(np.clip(np.dot(source, target), -1.0, 1.0))

    if dot > 1.0 - 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if dot < -1.0 + 1e-10:
        # 两向量反向时，选择任意与 source 正交的稳定旋转轴。
        axis = np.cross(source, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(source, np.array([0.0, 1.0, 0.0]))
        axis /= np.linalg.norm(axis)
        return np.array([0.0, axis[0], axis[1], axis[2]])

    cross = np.cross(source, target)
    return normalize_quaternion(np.array([1.0 + dot, *cross]))


def quaternion_to_rotation(quaternion):
    """将 wxyz 四元数转换为三维旋转矩阵。"""
    w, x, y, z = normalize_quaternion(quaternion)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def delta_quaternion(angular_velocity, dt):
    """根据机体系角速度和时间间隔生成增量四元数。"""
    angle = float(np.linalg.norm(angular_velocity) * dt)
    if angle < 1e-12:
        return normalize_quaternion(
            np.array([1.0, *(0.5 * angular_velocity * dt)])
        )
    axis = angular_velocity / np.linalg.norm(angular_velocity)
    half_angle = 0.5 * angle
    return np.array([math.cos(half_angle), *(axis * math.sin(half_angle))])


def integrate_imu_trajectory(stamps, accelerations, angular_velocities, initial_seconds):
    """使用初始重力对齐和陀螺仪积分，计算仅供诊断的纯 IMU 轨迹。"""
    relative_time = stamps - stamps[0]
    initial_mask = relative_time <= max(initial_seconds, 0.05)
    if np.count_nonzero(initial_mask) < 3:
        initial_mask[: min(20, len(stamps))] = True

    initial_acceleration = np.mean(accelerations[initial_mask], axis=0)
    gyro_bias = np.mean(angular_velocities[initial_mask], axis=0)

    # 静止时加速度计测到的比力应指向世界坐标 +Z，用它初始化横滚和俯仰。
    quaternion = quaternion_from_two_vectors(
        initial_acceleration, np.array([0.0, 0.0, STANDARD_GRAVITY])
    )
    positions = np.zeros((len(stamps), 3), dtype=np.float64)
    velocities = np.zeros((len(stamps), 3), dtype=np.float64)
    world_accelerations = np.zeros((len(stamps), 3), dtype=np.float64)

    for index in range(1, len(stamps)):
        dt = stamps[index] - stamps[index - 1]
        corrected_gyro = (
            0.5 * (angular_velocities[index - 1] + angular_velocities[index])
            - gyro_bias
        )

        # 先把姿态传播到区间中点，再用该姿态消除重力。
        half_delta = delta_quaternion(corrected_gyro, 0.5 * dt)
        midpoint_quaternion = normalize_quaternion(
            quaternion_multiply(quaternion, half_delta)
        )
        body_acceleration = 0.5 * (
            accelerations[index - 1] + accelerations[index]
        )
        world_specific_force = (
            quaternion_to_rotation(midpoint_quaternion) @ body_acceleration
        )
        world_acceleration = world_specific_force - np.array(
            [0.0, 0.0, STANDARD_GRAVITY]
        )
        world_accelerations[index] = world_acceleration

        # 使用匀加速度模型积分；偏置会被二次积分，因此轨迹只能用于短时诊断。
        positions[index] = (
            positions[index - 1]
            + velocities[index - 1] * dt
            + 0.5 * world_acceleration * dt * dt
        )
        velocities[index] = velocities[index - 1] + world_acceleration * dt
        quaternion = normalize_quaternion(
            quaternion_multiply(
                quaternion, delta_quaternion(corrected_gyro, dt)
            )
        )

    return positions, velocities, world_accelerations, gyro_bias


def find_shock_events(stamps, acceleration_norm, threshold_g, separation_seconds=0.1):
    """提取超过阈值且时间上彼此分离的局部最大冲击事件。"""
    threshold = threshold_g * STANDARD_GRAVITY
    candidates = np.flatnonzero(acceleration_norm >= threshold)
    if candidates.size == 0:
        return []

    events = []
    current_group = [int(candidates[0])]
    for index in candidates[1:]:
        if stamps[index] - stamps[current_group[-1]] <= separation_seconds:
            current_group.append(int(index))
        else:
            events.append(max(current_group, key=lambda i: acceleration_norm[i]))
            current_group = [int(index)]
    events.append(max(current_group, key=lambda i: acceleration_norm[i]))
    return events


def save_csv(path, relative_time, accelerations, angular_velocities, world_accelerations, positions, velocities):
    """保存每条 IMU 数据和诊断积分结果，便于后续自行筛选。"""
    values = np.column_stack(
        (
            relative_time,
            accelerations,
            np.linalg.norm(accelerations, axis=1),
            angular_velocities,
            np.linalg.norm(angular_velocities, axis=1),
            world_accelerations,
            positions,
            velocities,
        )
    )
    header = (
        "time_s,accel_x,accel_y,accel_z,accel_norm,"
        "gyro_x,gyro_y,gyro_z,gyro_norm,"
        "world_accel_x,world_accel_y,world_accel_z,"
        "position_x,position_y,position_z,"
        "velocity_x,velocity_y,velocity_z"
    )
    np.savetxt(path, values, delimiter=",", header=header, comments="")


def plot_analysis(output_path, relative_time, accelerations, angular_velocities, positions, velocities, event_indices):
    """绘制加速度、角速度、速度和纯 IMU 积分轨迹。"""
    acceleration_norm = np.linalg.norm(accelerations, axis=1)
    angular_velocity_norm = np.linalg.norm(angular_velocities, axis=1)
    speed = np.linalg.norm(velocities, axis=1)

    figure = plt.figure(figsize=(15, 10), constrained_layout=True)
    accel_axis = figure.add_subplot(2, 2, 1)
    gyro_axis = figure.add_subplot(2, 2, 2)
    speed_axis = figure.add_subplot(2, 2, 3)
    trajectory_axis = figure.add_subplot(2, 2, 4)

    labels = ("x", "y", "z")
    for axis_index, label in enumerate(labels):
        accel_axis.plot(
            relative_time, accelerations[:, axis_index], linewidth=0.8, label=label
        )
        gyro_axis.plot(
            relative_time,
            angular_velocities[:, axis_index],
            linewidth=0.8,
            label=label,
        )
    accel_axis.plot(relative_time, acceleration_norm, "k", linewidth=1.1, label="norm")
    for event_index in event_indices:
        accel_axis.scatter(
            relative_time[event_index],
            acceleration_norm[event_index],
            color="red",
            s=28,
            zorder=5,
        )
    accel_axis.axhline(STANDARD_GRAVITY, color="gray", linestyle="--", linewidth=0.8)
    accel_axis.set(title="Acceleration / impact", xlabel="time (s)", ylabel="m/s²")
    accel_axis.legend(ncol=4)
    accel_axis.grid(True, alpha=0.25)

    gyro_axis.plot(relative_time, angular_velocity_norm, "k", linewidth=1.1, label="norm")
    gyro_axis.set(title="Angular velocity", xlabel="time (s)", ylabel="rad/s")
    gyro_axis.legend(ncol=4)
    gyro_axis.grid(True, alpha=0.25)

    speed_axis.plot(relative_time, speed)
    speed_axis.set(
        title="Raw IMU integrated speed (diagnostic only)",
        xlabel="time (s)",
        ylabel="m/s",
    )
    speed_axis.grid(True, alpha=0.25)

    trajectory_axis.plot(positions[:, 0], positions[:, 1])
    trajectory_axis.scatter(
        positions[0, 0], positions[0, 1], color="green", label="start"
    )
    trajectory_axis.scatter(
        positions[-1, 0],
        positions[-1, 1],
        color="red",
        label="end",
    )
    trajectory_axis.set(
        title=(
            "Raw IMU double-integrated XY trajectory\n"
            f"final z drift = {positions[-1, 2]:.2f} m"
        ),
        xlabel="x (m)",
        ylabel="y (m)",
    )
    trajectory_axis.legend()
    trajectory_axis.axis("equal")
    trajectory_axis.grid(True, alpha=0.25)

    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def mark_shock_times(axis, shock_times, peak_time):
    """在时间曲线上标出全部冲击，并突出最大冲击。"""
    for shock_time in shock_times:
        axis.axvline(shock_time, color="red", alpha=0.12, linewidth=0.8)
    axis.axvline(
        peak_time,
        color="red",
        linestyle="--",
        linewidth=1.3,
        label=f"max shock @ {peak_time:.2f}s",
    )


def plot_vio_alignment(output_path, stamps, accelerations, event_indices, vio):
    """绘制 IMU 冲击、OpenVINS 轨迹、视觉状态和协方差对齐图。"""
    time_origin = stamps[0]
    imu_time = stamps - time_origin
    acceleration_g = np.linalg.norm(accelerations, axis=1) / STANDARD_GRAVITY
    shock_times = imu_time[event_indices]
    peak_index = int(np.argmax(acceleration_g))
    peak_time = imu_time[peak_index]

    odom_time = vio["odom_stamps"] - time_origin
    positions = vio["positions"]
    speed = np.linalg.norm(vio["velocities"], axis=1)
    position_stds = vio["position_stds"]
    odom_steps = np.concatenate(
        ([0.0], np.linalg.norm(np.diff(positions, axis=0), axis=1))
    )

    figure, axes = plt.subplots(2, 3, figsize=(18, 10), constrained_layout=True)
    acceleration_axis = axes[0, 0]
    position_axis = axes[0, 1]
    health_axis = axes[0, 2]
    speed_axis = axes[1, 0]
    trajectory_axis = axes[1, 1]
    step_axis = axes[1, 2]

    acceleration_axis.plot(imu_time, acceleration_g, linewidth=0.8)
    acceleration_axis.axhline(
        2.0, color="orange", linestyle="--", linewidth=1.0, label="2g threshold"
    )
    mark_shock_times(acceleration_axis, shock_times, peak_time)
    acceleration_axis.set(
        title="IMU resultant acceleration",
        xlabel="time (s)",
        ylabel="g",
    )
    acceleration_axis.legend()
    acceleration_axis.grid(True, alpha=0.25)

    for axis_index, label in enumerate(("x", "y", "z")):
        position_axis.plot(
            odom_time, positions[:, axis_index], linewidth=1.0, label=label
        )
    mark_shock_times(position_axis, shock_times, peak_time)
    position_axis.set(
        title="OpenVINS odometry position",
        xlabel="time (s)",
        ylabel="m",
    )
    position_axis.legend()
    position_axis.grid(True, alpha=0.25)

    if vio["info_stamps"].size:
        info_time = vio["info_stamps"] - time_origin
        health_axis.plot(
            info_time, vio["features"], label="update features", linewidth=1.0
        )
        health_axis.plot(
            info_time,
            vio["local_map_sizes"],
            label="active/local map tracks",
            linewidth=1.0,
        )
        lost_indices = np.flatnonzero(vio["lost_flags"])
        if lost_indices.size:
            health_axis.scatter(
                info_time[lost_indices],
                np.zeros(lost_indices.size),
                color="red",
                s=18,
                label="lost=true",
            )
    else:
        health_axis.text(
            0.5,
            0.5,
            "No /odom_info in bag",
            horizontalalignment="center",
            verticalalignment="center",
            transform=health_axis.transAxes,
        )
    mark_shock_times(health_axis, shock_times, peak_time)
    health_axis.set(
        title="Visual update health",
        xlabel="time (s)",
        ylabel="count",
    )
    health_axis.legend()
    health_axis.grid(True, alpha=0.25)

    speed_axis.plot(odom_time, speed, label="reported speed", linewidth=1.0)
    std_axis = speed_axis.twinx()
    std_axis.plot(
        odom_time,
        position_stds,
        color="purple",
        label="max position std",
        linewidth=1.0,
    )
    mark_shock_times(speed_axis, shock_times, peak_time)
    speed_axis.set(
        title="Velocity and position uncertainty",
        xlabel="time (s)",
        ylabel="speed (m/s)",
    )
    std_axis.set_ylabel("position std (m)", color="purple")
    # 手工合并双 Y 轴图例时，过滤 Matplotlib 自动生成的冲击辅助线标签。
    speed_lines = [
        line
        for line in speed_axis.get_lines() + std_axis.get_lines()
        if not line.get_label().startswith("_")
    ]
    speed_axis.legend(
        speed_lines,
        [line.get_label() for line in speed_lines],
        loc="upper left",
    )
    speed_axis.grid(True, alpha=0.25)

    trajectory_axis.plot(positions[:, 0], positions[:, 1], linewidth=1.0)
    trajectory_axis.scatter(
        positions[0, 0], positions[0, 1], color="green", label="start"
    )
    trajectory_axis.scatter(
        positions[-1, 0], positions[-1, 1], color="red", label="end"
    )
    trajectory_axis.set(
        title=f"OpenVINS XY trajectory, final z={positions[-1, 2]:.2f}m",
        xlabel="x (m)",
        ylabel="y (m)",
    )
    trajectory_axis.axis("equal")
    trajectory_axis.legend()
    trajectory_axis.grid(True, alpha=0.25)

    step_axis.plot(odom_time, odom_steps, linewidth=0.9, label="frame translation")
    if vio["info_stamps"].size:
        info_time = vio["info_stamps"] - time_origin
        interval_axis = step_axis.twinx()
        interval_axis.plot(
            info_time,
            vio["intervals"] * 1000.0,
            color="orange",
            alpha=0.65,
            linewidth=0.8,
            label="odom interval",
        )
        interval_axis.set_ylabel("interval (ms)", color="orange")
        step_lines = step_axis.get_lines() + interval_axis.get_lines()
    else:
        step_lines = step_axis.get_lines()
    mark_shock_times(step_axis, shock_times, peak_time)
    step_axis.set(
        title="Odometry jumps and timing",
        xlabel="time (s)",
        ylabel="translation per frame (m)",
    )
    step_axis.legend(
        step_lines,
        [line.get_label() for line in step_lines],
        loc="upper left",
    )
    step_axis.grid(True, alpha=0.25)

    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def print_vio_summary(stamps, accelerations, event_indices, vio):
    """输出里程计轨迹、异常跳变以及每次冲击附近的视觉状态。"""
    time_origin = stamps[0]
    imu_time = stamps - time_origin
    acceleration_norm = np.linalg.norm(accelerations, axis=1)
    odom_time = vio["odom_stamps"] - time_origin
    positions = vio["positions"]
    speed = np.linalg.norm(vio["velocities"], axis=1)
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)

    print("\n===== IMU 与 OpenVINS /odom 对齐结果 =====")
    print(
        "里程计起点: "
        f"[{positions[0, 0]:.3f}, {positions[0, 1]:.3f}, "
        f"{positions[0, 2]:.3f}] m"
    )
    print(
        "里程计终点: "
        f"[{positions[-1, 0]:.3f}, {positions[-1, 1]:.3f}, "
        f"{positions[-1, 2]:.3f}] m"
    )
    displacement = positions[-1] - positions[0]
    print(
        "终点相对起点: "
        f"[{displacement[0]:.3f}, {displacement[1]:.3f}, "
        f"{displacement[2]:.3f}] m"
    )
    print(f"累计里程计路径长度: {np.sum(steps):.3f} m")
    max_step_index = int(np.argmax(steps))
    print(
        f"最大单帧位置跳变: {steps[max_step_index]:.3f} m，"
        f"发生在 t={odom_time[max_step_index + 1]:.3f} s"
    )
    max_speed_index = int(np.argmax(speed))
    print(
        f"最大报告速度: {speed[max_speed_index]:.3f} m/s，"
        f"发生在 t={odom_time[max_speed_index]:.3f} s"
    )
    print(
        f"最大位置标准差: {np.max(vio['position_stds']):.3f} m，"
        f"结束时 {vio['position_stds'][-1]:.3f} m"
    )
    if vio["info_stamps"].size:
        print(
            f"视觉特征 min/median/max: {np.min(vio['features'])}/"
            f"{np.median(vio['features']):.0f}/{np.max(vio['features'])}"
        )
        print(f"lost=true 帧数: {np.count_nonzero(vio['lost_flags'])}")

    print("冲击时刻对应状态:")
    for order, event_index in enumerate(event_indices[:15], start=1):
        event_time = imu_time[event_index]
        odom_index = int(np.argmin(np.abs(odom_time - event_time)))
        feature_text = ""
        if vio["info_stamps"].size:
            info_time = vio["info_stamps"] - time_origin
            info_index = int(np.argmin(np.abs(info_time - event_time)))
            feature_text = (
                f", features={vio['features'][info_index]}, "
                f"local_map={vio['local_map_sizes'][info_index]}"
            )
        print(
            f"  {order:02d}. t={event_time:.3f}s, "
            f"{acceleration_norm[event_index] / STANDARD_GRAVITY:.3f}g, "
            f"p=[{positions[odom_index, 0]:.3f}, "
            f"{positions[odom_index, 1]:.3f}, "
            f"{positions[odom_index, 2]:.3f}], "
            f"speed={speed[odom_index]:.3f}m/s, "
            f"std={vio['position_stds'][odom_index]:.3f}m"
            f"{feature_text}"
        )


def print_summary(stamps, accelerations, angular_velocities, velocities, positions, gyro_bias, event_indices):
    """在终端输出采样频率、最大冲击和积分漂移摘要。"""
    relative_time = stamps - stamps[0]
    dt = np.diff(stamps)
    acceleration_norm = np.linalg.norm(accelerations, axis=1)
    angular_velocity_norm = np.linalg.norm(angular_velocities, axis=1)
    max_accel_index = int(np.argmax(acceleration_norm))
    max_gyro_index = int(np.argmax(angular_velocity_norm))

    print(f"IMU 数据量: {len(stamps)}")
    print(f"时长: {relative_time[-1]:.3f} s")
    print(f"采样频率中位数: {1.0 / np.median(dt):.2f} Hz")
    print(f"最大采样间隔: {np.max(dt) * 1000.0:.3f} ms")
    print(
        "最大加速度模长: "
        f"{acceleration_norm[max_accel_index]:.3f} m/s² = "
        f"{acceleration_norm[max_accel_index] / STANDARD_GRAVITY:.3f} g，"
        f"发生在 t={relative_time[max_accel_index]:.3f} s"
    )
    print(
        "相对静止重力的模长超量: "
        f"{max(0.0, acceleration_norm[max_accel_index] - STANDARD_GRAVITY):.3f} m/s²"
    )
    print(
        "最大角速度模长: "
        f"{angular_velocity_norm[max_gyro_index]:.3f} rad/s = "
        f"{math.degrees(angular_velocity_norm[max_gyro_index]):.1f} deg/s，"
        f"发生在 t={relative_time[max_gyro_index]:.3f} s"
    )
    print(
        "初始陀螺仪零偏估计: "
        f"[{gyro_bias[0]:.6f}, {gyro_bias[1]:.6f}, {gyro_bias[2]:.6f}] rad/s"
    )
    print(f"超过阈值的独立冲击事件: {len(event_indices)} 个")
    for order, index in enumerate(event_indices[:10], start=1):
        print(
            f"  {order:02d}. t={relative_time[index]:.3f} s, "
            f"|a|={acceleration_norm[index]:.3f} m/s² "
            f"({acceleration_norm[index] / STANDARD_GRAVITY:.3f} g)"
        )
    print(
        "纯 IMU 积分终点（只用于看漂移趋势）: "
        f"[{positions[-1, 0]:.3f}, {positions[-1, 1]:.3f}, "
        f"{positions[-1, 2]:.3f}] m"
    )
    print(f"纯 IMU 积分末速度: {np.linalg.norm(velocities[-1]):.3f} m/s")


def main():
    """执行 IMU 分析，并在 bag 含 /odom 时自动生成对齐分析。"""
    args = parse_args()
    bag_path = args.bag.expanduser().resolve()
    output_path = (
        args.output.expanduser().resolve()
        if args.output
        else bag_path / "imu_analysis.png"
    )
    csv_path = (
        args.csv.expanduser().resolve()
        if args.csv
        else bag_path / "imu_samples.csv"
    )
    vio_output_path = (
        args.vio_output.expanduser().resolve()
        if args.vio_output
        else bag_path / "vio_imu_timeline.png"
    )

    storage_id, topic_types = load_bag_metadata(bag_path)
    topic_name = select_imu_topic(topic_types, args.topic)
    stamps, accelerations, angular_velocities = read_imu_samples(
        bag_path, storage_id, topic_name
    )
    positions, velocities, world_accelerations, gyro_bias = integrate_imu_trajectory(
        stamps, accelerations, angular_velocities, args.initial_seconds
    )
    acceleration_norm = np.linalg.norm(accelerations, axis=1)
    event_indices = find_shock_events(
        stamps, acceleration_norm, max(args.shock_g, 0.0)
    )
    relative_time = stamps - stamps[0]

    save_csv(
        csv_path,
        relative_time,
        accelerations,
        angular_velocities,
        world_accelerations,
        positions,
        velocities,
    )
    plot_analysis(
        output_path,
        relative_time,
        accelerations,
        angular_velocities,
        positions,
        velocities,
        event_indices,
    )
    print(f"IMU topic: {topic_name}")
    print_summary(
        stamps,
        accelerations,
        angular_velocities,
        velocities,
        positions,
        gyro_bias,
        event_indices,
    )
    print(f"分析图: {output_path}")
    print(f"样本 CSV: {csv_path}")

    if topic_types.get(args.odom_topic) == "nav_msgs/msg/Odometry":
        vio = read_vio_samples(
            bag_path,
            storage_id,
            args.odom_topic,
            args.odom_info_topic,
        )
        plot_vio_alignment(
            vio_output_path,
            stamps,
            accelerations,
            event_indices,
            vio,
        )
        print_vio_summary(stamps, accelerations, event_indices, vio)
        print(f"IMU/里程计对齐图: {vio_output_path}")
    else:
        print(
            f"未生成里程计对齐图：bag 中不存在 "
            f"{args.odom_topic} (nav_msgs/msg/Odometry)"
        )


if __name__ == "__main__":
    main()
