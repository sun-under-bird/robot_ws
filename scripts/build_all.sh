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

# 加载刚完成构建的工作空间，使后续工作空间能发现其运行依赖。
SourceWorkspace() {
    local workspace_path="$1"
    local setup_file="${workspace_path}/install/local_setup.bash"

    if [[ ! -f "${setup_file}" ]]; then
        echo "Workspace setup not found: ${setup_file}" >&2
        return 1
    fi

    # shellcheck disable=SC1090
    source "${setup_file}"
}

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "ROS 2 environment not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
    exit 1
fi

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# 自编译 RTAB-Map 是外部依赖，只加载当前前缀，避免继承它历史记录中的旧工作区。
if [[ -n "${ROBOT_RTABMAP_WS:-}" && \
      -f "${ROBOT_RTABMAP_WS}/install/local_setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${ROBOT_RTABMAP_WS}/install/local_setup.bash"
fi

BuildWorkspace "sensor_ws" "${ROOT_DIR}/sensor_ws"
SourceWorkspace "${ROOT_DIR}/sensor_ws"

BuildWorkspace "openvins_ws" "${ROOT_DIR}/openvins_ws"
SourceWorkspace "${ROOT_DIR}/openvins_ws"

BuildWorkspace "VINS-Fusion-ROS2-humble-arm" "${ROOT_DIR}/VINS-Fusion-ROS2-humble-arm"
SourceWorkspace "${ROOT_DIR}/VINS-Fusion-ROS2-humble-arm"

# 建图导航依赖传感器驱动及前端算法，因此放在最后构建。
BuildWorkspace "slam_ws" "${ROOT_DIR}/slam_ws"

echo "[robot_ws] All workspaces built successfully"
