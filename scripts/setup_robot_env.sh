#!/usr/bin/env bash

# 统一设置仓库根目录和运行输出目录；允许调用方预先覆盖这两个变量。
_robot_ws_detected_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROBOT_WS_ROOT="${ROBOT_WS_ROOT:-${_robot_ws_detected_root}}"
export ROBOT_OUTPUT_DIR="${ROBOT_OUTPUT_DIR:-${ROBOT_WS_ROOT}/output}"
export ROS_DISTRO="${ROS_DISTRO:-humble}"

# VINS 和离线重建工具会直接写这些目录。
mkdir -p "${ROBOT_OUTPUT_DIR}" \
         "${ROBOT_OUTPUT_DIR}/pose_graph" \
         "${ROBOT_OUTPUT_DIR}/dense_recon"

unset _robot_ws_detected_root

