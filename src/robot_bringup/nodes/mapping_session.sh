#!/usr/bin/env bash
# 受控建图入口：Ctrl+C 时先锁止控制，再让 Cartographer 退出执行最终优化，最后原子提交 bundle。
set -u

workspace="${CAR01_WORKSPACE:-$(cd "$(dirname "$0")/../../.." && pwd)}"
bundle_root="${workspace}/src/robot_slam/map/bundles"
map_id="map-$(date -u +%Y%m%dT%H%M%SZ)"
staging="${bundle_root}/.staging-${map_id}"
final_state="${staging}/.final.pbstream.tmp"
launch_pid=""
readonly_pid=""
finishing=0

cleanup() {
  [[ -n "${readonly_pid}" ]] && kill "${readonly_pid}" 2>/dev/null || true
  [[ -n "${launch_pid}" ]] && kill -INT "${launch_pid}" 2>/dev/null || true
}

finish_session() {
  [[ "${finishing}" -eq 1 ]] && return
  finishing=1
  trap '' INT TERM
  echo "[mapping_session] 锁止所有控制输入..."
  if ! rosservice call /control/stop '{}' >/tmp/car01_mapping_stop.out 2>&1; then
    echo "[mapping_session] /control/stop 未确认；立即结束 launch，保留上一个 active bundle。" >&2
    cleanup
    exit 2
  fi
  echo "[mapping_session] 冻结轨迹与 B/末位姿锚点..."
  if ! rosservice call /mapping/prepare_finalize '{}' | tee /tmp/car01_mapping_prepare.out | grep -q 'success: True'; then
    echo "[mapping_session] prepare 失败；结束 launch，active bundle 不变。" >&2
    cleanup
    exit 3
  fi
  rosnode kill /cartographer_node >/dev/null 2>&1 || true
  for _ in $(seq 1 120); do
    [[ -s "${final_state}" ]] && break
    sleep 1
  done
  if [[ ! -s "${final_state}" ]]; then
    echo "[mapping_session] Cartographer 最终优化文件未生成；active bundle 不变。" >&2
    cleanup
    exit 4
  fi

  echo "[mapping_session] 只读加载最终 pbstream，重投影锚点..."
  ROS_NAMESPACE=/wincodex_finalize rosrun cartographer_ros cartographer_node \
    -configuration_directory "${workspace}/src/robot_slam/config" \
    -configuration_basename localization.lua \
    -load_state_filename "${final_state}" \
    -load_frozen_state true \
    -start_trajectory_with_default_topics false >/tmp/car01_mapping_readonly.log 2>&1 &
  readonly_pid=$!
  for _ in $(seq 1 30); do
    rosservice info /wincodex_finalize/trajectory_query >/dev/null 2>&1 && break
    sleep 1
  done
  if ! rosservice call /mapping/commit_final '{}' | tee /tmp/car01_mapping_commit.out | grep -q 'success: True'; then
    echo "[mapping_session] 最终 bundle 提交失败；active bundle 不变。" >&2
    cleanup
    exit 5
  fi
  echo "[mapping_session] bundle 提交成功：${map_id}"
  cleanup
  exit 0
}

mkdir -p "${staging}"
trap finish_session INT TERM
trap cleanup EXIT
roslaunch robot_bringup bringup_mapping.launch \
  map_id:="${map_id}" staging_dir:="${staging}" final_state_file:="${final_state}" &
launch_pid=$!
wait "${launch_pid}"
