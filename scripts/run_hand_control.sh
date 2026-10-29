#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/runtime_environment.sh"
python scripts/check_camera.py --output artifacts/install/camera_report.json
exec ros2 launch omx_bringup bringup.launch.py mode:=servo hand:=true "$@"
