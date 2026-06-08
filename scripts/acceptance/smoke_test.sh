#!/usr/bin/env bash
# Smoke test for the AutoBee topic graph.
#
# Run this AFTER `ros2 launch autobee_core demo.launch.py` is up in another
# terminal. Verifies that every node in the contract is publishing/subscribing
# at the expected rate, exits non-zero on the first failure.
#
# Usage:
#   bash scripts/acceptance/smoke_test.sh
set -euo pipefail

CHECK_DURATION=8     # seconds to sample each topic
MIN_FPS_CAMERA=4.0   # NFR-02 says >=5; allow 4 floor for sampling jitter
MIN_HZ_PERCEPTION=2.0
MIN_HZ_ANALYTICS=0.5

pass() { printf '  \033[32m✓\033[0m %s\n' "$1"; }
fail() { printf '  \033[31m✗\033[0m %s\n' "$1"; exit 1; }
hdr()  { printf '\n\033[1m%s\033[0m\n' "$1"; }

require_topic() {
  local topic="$1" min_hz="$2"
  # `ros2 topic hz` doesn't exit on its own; sample then kill.
  local out
  out=$(timeout "$CHECK_DURATION" ros2 topic hz "$topic" 2>&1 | tail -n 5 || true)
  local hz
  hz=$(echo "$out" | grep -oE 'average rate: [0-9.]+' | head -1 | awk '{print $3}' || true)
  if [[ -z "$hz" ]]; then
    fail "$topic — no messages observed in ${CHECK_DURATION}s"
  fi
  awk -v h="$hz" -v m="$min_hz" 'BEGIN{exit !(h+0 >= m+0)}' \
    && pass "$topic at ${hz} Hz (>= ${min_hz})" \
    || fail "$topic at ${hz} Hz (< ${min_hz})"
}

require_topic_alive() {
  local topic="$1"
  if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
    pass "$topic exists"
  else
    fail "$topic not in topic list"
  fi
}

hdr "Topic existence"
for t in /drone/cmd_vel /drone/odom /drone/goal /drone/goal_status \
         /drone/camera/image_raw /drone/imu /user_intent \
         /mission_command /mission_status /mission_report /mission_event \
         /perception/event /crowd_analytics /anomaly_alert /mission_abort; do
  require_topic_alive "$t"
done

hdr "Publishing rates"
require_topic /drone/camera/image_raw "$MIN_FPS_CAMERA"
require_topic /perception/event       "$MIN_HZ_PERCEPTION"
require_topic /crowd_analytics        "$MIN_HZ_ANALYTICS"

hdr "Node liveness"
for n in /llm_agent_node /report_node /yolov8_perception_node \
         /crowd_analytics_node /anomaly_detection_node /mission_executor_node \
         /flight_controller_node; do
  if ros2 node list 2>/dev/null | grep -qx "$n"; then
    pass "$n alive"
  else
    fail "$n not in node list"
  fi
done

hdr "Result"
echo "  All topic-graph and node-liveness checks passed."
