#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/runtime_environment.sh"
exec ros2 launch omx_bringup bringup.launch.py mode:=servo hand:=false "$@"
