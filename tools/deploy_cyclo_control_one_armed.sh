#!/usr/bin/env bash
# Deploy the one-armed cyclo_control controllers into the ai_worker container.
#
# Why this exists: ~/cyclo_control on the host is NOT mounted into any container. The ai_worker
# container has its own copy at /root/ros2_ws/src/cyclo_control (baked into the image), so host
# changes only take effect after they are copied in and built there.
#
# What it does, in order (stops at the first failure):
#   1. refuses to run if a cyclo_control controller or the follower bringup is running;
#   2. checks that every file it is about to overwrite in the container is identical to the host
#      BASE branch (tabletop-cartesian-teleop) or absent, so no container-only edit is lost;
#   3. saves the container's current copies of those files to
#      /root/cyclo_control_backup_<timestamp>.tar inside the container;
#   4. copies the changed files from the host working tree (branch one-armed) into the container;
#   5. builds cyclo_motion_controller_ros inside the container (about 5 minutes);
#   6. prints how to verify and how to roll back.
#
# It does not start, stop or arm anything on the robot.
#
# Usage:   ~/ai_worker/tools/deploy_cyclo_control_one_armed.sh            # dry run: checks only
#          ~/ai_worker/tools/deploy_cyclo_control_one_armed.sh --apply    # copy + build
#
# NOTE: the result lives only inside the container. If the ai_worker container is ever recreated
# (docker compose up/down, image update), rerun this script.
set -euo pipefail

HOST_REPO="${HOST_REPO:-$HOME/cyclo_control}"
BASE_BRANCH="tabletop-cartesian-teleop"
EXPECT_BRANCH="one-armed"
CONTAINER="ai_worker"
DEST="/root/ros2_ws/src/cyclo_control"
APPLY=0
[ "${1:-}" = "--apply" ] && APPLY=1

cd "$HOST_REPO"
branch="$(git branch --show-current)"
if [ "$branch" != "$EXPECT_BRANCH" ]; then
  echo "ERROR: $HOST_REPO is on branch '$branch', expected '$EXPECT_BRANCH'." >&2; exit 1
fi

# Files that differ from the base branch (tracked changes + new files under the ros package).
mapfile -t FILES < <( { git diff --name-only "$BASE_BRANCH" -- cyclo_motion_controller_ros cyclo_motion_controller_core cyclo_motion_controller_models
                        git ls-files --others --exclude-standard -- cyclo_motion_controller_ros cyclo_motion_controller_core cyclo_motion_controller_models; } | sort -u )
if [ "${#FILES[@]}" -eq 0 ]; then echo "Nothing to deploy."; exit 0; fi
echo "Files to deploy (${#FILES[@]}):"; printf '  %s\n' "${FILES[@]}"

# 1. nothing may be running
running="$(docker exec "$CONTAINER" bash -c "ps -eo comm,args | grep -E 'controller_node|ros2_control_no|ai_worker_controller.launch' | grep -v grep || true")"
if [ -n "$running" ]; then
  echo "ERROR: these are running in $CONTAINER; stop the controller and the bringup first:" >&2
  echo "$running" >&2; exit 1
fi
echo "OK: no controller or bringup running."

# 2. container copies must equal the host base branch (or be absent)
drift=0
for f in "${FILES[@]}"; do
  if docker exec "$CONTAINER" test -e "$DEST/$f"; then
    if git cat-file -e "$BASE_BRANCH:$f" 2>/dev/null; then
      if ! docker exec "$CONTAINER" cat "$DEST/$f" | cmp -s - <(git show "$BASE_BRANCH:$f"); then
        echo "DRIFT: container copy of $f differs from $BASE_BRANCH"; drift=1
      fi
    else
      echo "DRIFT: $f exists in the container but not on $BASE_BRANCH"; drift=1
    fi
  fi
done
if [ "$drift" -ne 0 ]; then
  echo "ERROR: the container has changes this script would overwrite. Resolve them by hand first." >&2; exit 1
fi
echo "OK: container copies match $BASE_BRANCH; nothing container-only will be lost."

if [ "$APPLY" -ne 1 ]; then
  echo; echo "Dry run only. Re-run with --apply to copy the files and build."; exit 0
fi

# 3. backup
stamp="$(date +%Y%m%d_%H%M%S)"
backup="/root/cyclo_control_backup_${stamp}.tar"
existing=()
for f in "${FILES[@]}"; do docker exec "$CONTAINER" test -e "$DEST/$f" && existing+=("$f"); done
if [ "${#existing[@]}" -gt 0 ]; then
  docker exec "$CONTAINER" tar -C "$DEST" -cf "$backup" "${existing[@]}"
  echo "Backup of ${#existing[@]} files: $CONTAINER:$backup"
fi
printf '%s\n' "${FILES[@]}" | docker exec -i "$CONTAINER" tee "/root/cyclo_control_deployed_${stamp}.txt" >/dev/null

# 4. copy
for f in "${FILES[@]}"; do
  docker exec "$CONTAINER" mkdir -p "$DEST/$(dirname "$f")"
  docker cp "$HOST_REPO/$f" "$CONTAINER:$DEST/$f"
done
echo "Copied ${#FILES[@]} files."

# 5. build (only the ros package and what it needs that changed)
pkgs="cyclo_motion_controller_ros"
printf '%s\n' "${FILES[@]}" | grep -q '^cyclo_motion_controller_core/' && pkgs="cyclo_motion_controller_core $pkgs"
printf '%s\n' "${FILES[@]}" | grep -q '^cyclo_motion_controller_models/' && pkgs="cyclo_motion_controller_models $pkgs"
echo "Building: $pkgs (this takes several minutes)"
docker exec "$CONTAINER" bash -ic "cd /root/ros2_ws && colcon build --packages-select $pkgs 2>&1 | tail -15"

# 6. next steps
cat <<EOF

Deployed. Verify before using the robot:
  - start a controller with MOCK hardware first and check its log has:
      "enable_right_arm is false: right arm is not commanded and is modelled at its measured pose."
  - docker exec $CONTAINER grep -c enable_right_arm $DEST/cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp

Roll back:
  docker exec $CONTAINER bash -c 'cd $DEST && tar -xf $backup'      # restore the old sources
  (new files listed in /root/cyclo_control_deployed_${stamp}.txt that are not in the tar: delete them)
  docker exec $CONTAINER bash -ic 'cd /root/ros2_ws && colcon build --packages-select $pkgs'
EOF
