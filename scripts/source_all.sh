#!/usr/bin/env bash

# 统一加载 ROS 2 以及仓库内所有工作空间；本脚本应通过 source 调用。
_robot_ws_source_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_robot_ws_source_distro="${ROS_DISTRO:-humble}"

# shellcheck disable=SC1091
source "${_robot_ws_source_root}/scripts/setup_robot_env.sh"

if [[ ! -f "/opt/ros/${_robot_ws_source_distro}/setup.bash" ]]; then
    echo "ROS 2 environment not found: /opt/ros/${_robot_ws_source_distro}/setup.bash" >&2
    return 1 2>/dev/null || exit 1
fi

# 先加载基础 ROS，再按依赖方向依次叠加传感器、算法和建图导航工作空间。
# shellcheck disable=SC1090
source "/opt/ros/${_robot_ws_source_distro}/setup.bash"

# 外部 RTAB-Map 不属于本仓库；只加载其本地前缀，避免旧 underlay 路径污染环境。
if [[ -n "${ROBOT_RTABMAP_WS:-}" && \
      -f "${ROBOT_RTABMAP_WS}/install/local_setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${ROBOT_RTABMAP_WS}/install/local_setup.bash"
fi

if ! ros2 pkg prefix rtabmap_odom >/dev/null 2>&1; then
    echo "RTAB-Map not found; install it or set ROBOT_RTABMAP_WS." >&2
fi

for _robot_ws_source_name in \
    sensor_ws \
    openvins_ws \
    VINS-Fusion-ROS2-humble-arm \
    slam_ws; do
    _robot_ws_source_setup="${_robot_ws_source_root}/${_robot_ws_source_name}/install/local_setup.bash"
    if [[ -f "${_robot_ws_source_setup}" ]]; then
        # shellcheck disable=SC1090
        source "${_robot_ws_source_setup}"
    fi
done

unset _robot_ws_source_root
unset _robot_ws_source_distro
unset _robot_ws_source_name
unset _robot_ws_source_setup
