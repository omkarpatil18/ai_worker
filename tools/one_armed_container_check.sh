#!/usr/bin/env bash
# Pre-flight check (and repair) of the one-armed setup inside the ai_worker container.
#
# Why this exists: the container's install space holds COPIES of the ffw_bringup launch/config
# files, and its own copy of cyclo_control. They were put there by hand, so they are lost if the
# container is ever recreated (docker compose up/down, image update). After a recreate the
# standard bringup command would silently be the stock two-arm one again and power the right arm.
#
# Usage:
#   ~/ai_worker/tools/one_armed_container_check.sh             # check only; exit 0 = all good
#   ~/ai_worker/tools/one_armed_container_check.sh --install   # re-copy the bringup files, then check
#
# Run the check before the first bringup after ANY container restart or recreate.
# It does not start, stop or command anything on the robot.
set -uo pipefail

C="ai_worker"
SRC="/root/ros2_ws/src/ai_worker"                       # host ~/ai_worker, bind-mounted
IB="/root/ros2_ws/install/ffw_bringup/share/ffw_bringup"
ID="/root/ros2_ws/install/ffw_description/share/ffw_description"
XACRO_REL="ros2_control/ffw_sg2_rev1_follower/ffw_sg2_follower_left_arm.ros2_control.xacro"
COPIES=(
  "launch/ffw_sg2_follower_ai.launch.py"
  "launch/ffw_sg2_follower_ai_both_arms.launch.py"
  "config/ffw_sg2_rev1_follower/tabletop_manip_left_arm_position.yaml"
)

if [ "${1:-}" = "--install" ]; then
  if docker exec "$C" bash -c "ps -eo comm | grep -q '^ros2_control_no'"; then
    echo "ERROR: the bringup is running; stop it before installing." >&2; exit 1
  fi
  for f in "${COPIES[@]}"; do
    docker exec "$C" bash -c "mkdir -p '$IB/$(dirname "$f")' && cp '$SRC/ffw_bringup/$f' '$IB/$f'" && echo "installed $f"
  done
  docker exec "$C" bash -c "mkdir -p '$ID/$(dirname "$XACRO_REL")' && ln -sfn '$SRC/ffw_description/$XACRO_REL' '$ID/$XACRO_REL'" && echo "linked $XACRO_REL"
fi

fail=0
ok()   { echo "PASS  $1"; }
bad()  { echo "FAIL  $1"; fail=1; }
check() { if docker exec "$C" bash -c "$2" >/dev/null 2>&1; then ok "$1"; else bad "$1"; fi; }

for f in "${COPIES[@]}"; do
  check "installed $f matches the source" "cmp -s '$SRC/ffw_bringup/$f' '$IB/$f'"
done
check "standard bringup forces the left-arm servo config" \
  "grep -q \"'ros2_control_type:=', LEFT_ARM_ROS2_CONTROL_TYPE\" '$IB/launch/ffw_sg2_follower_ai.launch.py'"
check "standard bringup does not spawn arm_r_controller" \
  "! grep -q \"'arm_r_controller'\" '$IB/launch/ffw_sg2_follower_ai.launch.py'"
check "left-arm servo config is installed" "test -r '$ID/$XACRO_REL'"
check "left-arm servo config has Torque Enable 0 on 8 servos" \
  "[ \"\$(grep -c 'name=\"Torque Enable\">0<' '$ID/$XACRO_REL')\" = 8 ]"
check "robot description defaults to the left-arm servo config" \
  "grep -q 'name=\"ros2_control_type\" default=\"ffw_sg2_follower_left_arm\"' '$ID/urdf/ffw_sg2_rev1_follower/ffw_sg2_follower.urdf.xacro'"
check "cyclo_control in the container is the one-armed build (source has enable_right_arm)" \
  "grep -q enable_right_arm /root/ros2_ws/src/cyclo_control/cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp"
check "cyclo_control one-armed build is installed (binary has enable_right_arm)" \
  "grep -q enable_right_arm /root/ros2_ws/install/cyclo_motion_controller_ros/lib/cyclo_motion_controller_ros/vr_controller_node"

# Informational: arguments the Cyclo Manager would reuse for 'Robot start'.
args="$(docker exec "$C" cat /run/launch_args/ai_worker_bringup 2>/dev/null || true)"
if [ -n "$args" ]; then
  echo "INFO  saved Cyclo Manager bringup args: $args"
  echo "      (a ros2_control_type value there is ignored by the one-armed bringup)"
fi

echo
if [ "$fail" -eq 0 ]; then
  echo "All one-armed checks passed."
else
  echo "One or more checks FAILED."
  echo "  A bringup / servo-config failure: do NOT start the robot. Rerun with --install."
  echo "  A cyclo_control failure: the bringup is fine, but do NOT run any motion controller"
  echo "  (teleop, LLM agent). Deploy with ~/ai_worker/tools/deploy_cyclo_control_one_armed.sh --apply"
fi
exit "$fail"
