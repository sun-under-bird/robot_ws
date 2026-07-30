#!/usr/bin/env bash

# 统一设置仓库根目录和运行输出目录；允许调用方预先覆盖这两个变量。
_robot_ws_detected_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROBOT_WS_ROOT="${ROBOT_WS_ROOT:-${_robot_ws_detected_root}}"
export ROBOT_OUTPUT_DIR="${ROBOT_OUTPUT_DIR:-${ROBOT_WS_ROOT}/output}"
export ROS_DISTRO="${ROS_DISTRO:-humble}"

# 自动发现仓库同级的自编译 RTAB-Map；调用方也可以显式覆盖该路径。
_robot_ws_parent_dir="$(dirname "${ROBOT_WS_ROOT}")"
_robot_ws_default_rtabmap="${_robot_ws_parent_dir}/rtabmap_humble_ws"
if [[ -z "${ROBOT_RTABMAP_WS:-}" && -d "${_robot_ws_default_rtabmap}" ]]; then
    export ROBOT_RTABMAP_WS="${_robot_ws_default_rtabmap}"
fi

# VINS 和离线重建工具会直接写这些目录。
mkdir -p "${ROBOT_OUTPUT_DIR}" \
         "${ROBOT_OUTPUT_DIR}/pose_graph" \
         "${ROBOT_OUTPUT_DIR}/dense_recon"

unset _robot_ws_detected_root
unset _robot_ws_parent_dir
unset _robot_ws_default_rtabmap
