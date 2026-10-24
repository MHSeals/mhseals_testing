#!/usr/bin/env bash
# Usage: run_passive_startup.sh ABSOLUTE_PRIVATE_INSTALL ABSOLUTE_CHECKER EVIDENCE_DIR
# Prefix must be a real (not symlink-install) approved four-package Jazzy build.
set -euo pipefail
prefix=$(realpath "$1")
checker=$(realpath "$2")
mkdir -p "$3"
evidence=$(realpath "$3")
script=$(realpath "$(dirname "$0")/check_passive_startup.py")
image=sha256:4c29b81ffed7722267bf9eb41e1dad8d0b436f2be764c5c8a5174d4dfbfd48b3
name="c07-passive-$$"
container=$(docker run -d --name "$name" --network none --cap-drop ALL \
  --security-opt no-new-privileges --env ROS_DOMAIN_ID=227 \
  --env ROS_LOCALHOST_ONLY=1 "$image" sleep infinity)
finish() {
  docker cp "$container:/scratch/." "$evidence" || true
  docker inspect "$container" > "$evidence/container-inspect.json" || true
  docker rm -f "$container" >/dev/null || true
}
trap finish EXIT
docker exec -u 0 "$container" mkdir /scratch
docker exec -u 0 "$container" chmod 777 /scratch
docker cp "$prefix" "$container:/scratch/private-install"
docker cp "$checker" "$container:/scratch/check_config_install.py"
docker cp "$script" "$container:/scratch/check_passive_startup.py"
docker exec "$container" env -i HOME=/scratch/home XDG_CONFIG_HOME=/scratch/config \
  XDG_DATA_HOME=/scratch/data PATH=/usr/bin:/bin ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 \
  bash --noprofile --norc -c 'source /opt/ros/jazzy/setup.bash
    source /scratch/private-install/local_setup.bash
    cd /scratch
    dpkg-query -W ros-jazzy-nav2-planner ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-smac-planner > versions.log
    python3 check_config_install.py --prefix /scratch/private-install --include-testing > origins.json &&
    timeout --signal=INT --kill-after=15 150 python3 check_passive_startup.py > gate.log 2>&1'
