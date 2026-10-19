#!/usr/bin/env bash
# Open the bundled AgileX ROS helper scripts from any checkout location.
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir/pico_tele_agx/src/agx_arm_ros/scripts" || exit 1
bash can_muti_activate.sh
exec /bin/bash --rcfile "$HOME/.bashrc" -i
