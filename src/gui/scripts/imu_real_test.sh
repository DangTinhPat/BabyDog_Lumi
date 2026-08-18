#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SERIAL_DEV="${1:-${BABYDOG_IMU_SERIAL_DEV:-/dev/ttyUSB0}}"
readonly SERIAL_BAUD="${2:-${BABYDOG_IMU_SERIAL_BAUD:-921600}}"
readonly MROS_ROOT="${BABYDOG_MROS_WS:-${HOME}/mros/mros_ws}"

source_tree_setup="${SCRIPT_DIR}/../../../install/setup.bash"
installed_setup="${SCRIPT_DIR}/../../../../setup.bash"
if [[ -n "${BABYDOG_WS_SETUP:-}" ]]; then
    readonly BABYDOG_SETUP="${BABYDOG_WS_SETUP}"
elif [[ -f "${source_tree_setup}" ]]; then
    readonly BABYDOG_SETUP="${source_tree_setup}"
elif [[ -f "${installed_setup}" ]]; then
    readonly BABYDOG_SETUP="${installed_setup}"
else
    echo "ERROR: Không tìm thấy install/setup.bash của workspace babyDog." >&2
    exit 1
fi

if [[ ! -c "${SERIAL_DEV}" ]]; then
    echo "ERROR: ${SERIAL_DEV} không tồn tại hoặc không phải character device." >&2
    exit 1
fi
if [[ ! "${SERIAL_BAUD}" =~ ^[0-9]+$ || "${SERIAL_BAUD}" == "0" ]]; then
    echo "ERROR: Baud rate phải là số nguyên dương, nhận '${SERIAL_BAUD}'." >&2
    exit 1
fi
if [[ ! -r "${SERIAL_DEV}" || ! -w "${SERIAL_DEV}" ]]; then
    echo "ERROR: Không có quyền đọc/ghi ${SERIAL_DEV}. Kiểm tra group dialout." >&2
    exit 1
fi
if ! command -v lsof >/dev/null 2>&1; then
    echo "ERROR: Cần lsof để xác minh độc quyền cổng serial trước khi test thật." >&2
    exit 1
fi
if lsof "${SERIAL_DEV}" >/dev/null 2>&1; then
    echo "ERROR: ${SERIAL_DEV} đang bị tiến trình khác giữ:" >&2
    lsof "${SERIAL_DEV}" >&2
    exit 1
fi

active_real_processes="$(
    pgrep -af \
        '[m]icro_ros_agent|[r]os2_control_node|[r]obot_state_publisher|[r]eal_ros2_control\.launch\.py' \
        || true
)"
if [[ -n "${active_real_processes}" ]]; then
    echo "ERROR: Phát hiện stack robot thật/agent đang chạy; không mở pipeline IMU thứ hai:" >&2
    echo "${active_real_processes}" >&2
    exit 1
fi

for setup_file in \
    "/opt/ros/jazzy/setup.bash" \
    "${MROS_ROOT}/install/setup.bash" \
    "${BABYDOG_SETUP}"
do
    if [[ ! -f "${setup_file}" ]]; then
        echo "ERROR: Thiếu ${setup_file}. Hãy build đúng workspace trước." >&2
        exit 1
    fi
done

# ament/colcon-generated setup.bash files reference their own env vars (e.g.
# AMENT_TRACE_SETUP_FILES) without safe defaults - they are not written to be
# `set -u` safe, so sourcing them with nounset on throws "unbound variable"
# and aborts this script before `ros2 launch` ever runs. Disable -u only for
# the sourcing itself, matching the standard ROS tooling workaround.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "${MROS_ROOT}/install/setup.bash"
# shellcheck disable=SC1090
source "${BABYDOG_SETUP}"
set -u
export ROS_DOMAIN_ID=0

echo "IMU-only: ${SERIAL_DEV} @ ${SERIAL_BAUD}; không khởi động ros2_control."
exec ros2 launch gui imu_real_test.launch.py \
    serial_dev:="${SERIAL_DEV}" serial_baud:="${SERIAL_BAUD}"
