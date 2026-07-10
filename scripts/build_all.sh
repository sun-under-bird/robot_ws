#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/setup_robot_env.sh"

# 构建一个独立 ROS 2 工作空间，顺序执行可降低 RK3588 首次构建时的内存压力。
BuildWorkspace() {
    local workspace_name="$1"
    local workspace_path="$2"

    echo "[robot_ws] Building ${workspace_name}"
    (
        cd "${workspace_path}"
        colcon build --symlink-install --executor sequential
    )
}

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "ROS 2 environment not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
    exit 1
fi

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

BuildWorkspace "camera" "${ROOT_DIR}/camera"
BuildWorkspace "imu_ws" "${ROOT_DIR}/imu_ws"
BuildWorkspace "openvins_ws" "${ROOT_DIR}/openvins_ws"
BuildWorkspace "VINS-Fusion-ROS2-humble-arm" "${ROOT_DIR}/VINS-Fusion-ROS2-humble-arm"

echo "[robot_ws] All workspaces built successfully"
