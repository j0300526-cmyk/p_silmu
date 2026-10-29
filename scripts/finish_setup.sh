#!/usr/bin/env bash
# Unprivileged continuation after the system installer. Never run the build as root.
set -eo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
if [[ "$(id -u)" == 0 ]]; then
  echo "Run finish_setup.sh as your normal desktop user." >&2
  exit 1
fi
if [[ "${1:-}" == "--wait-install-pid" ]]; then
  installer_pid="${2:?Missing installer PID}"
  echo "Waiting for administrator-authenticated system installation (PID $installer_pid)."
  deadline=$((SECONDS + 1800))
  while ps -p "$installer_pid" >/dev/null 2>&1; do
    if (( SECONDS >= deadline )); then
      echo "System installation timed out; inspect artifacts/install/system_install.log" >&2
      exit 1
    fi
    sleep 2
  done
  if ! grep -q 'OMX_SYSTEM_INSTALL_COMPLETE' artifacts/install/system_install.log; then
    echo "System installation did not complete. Inspect system_install.log and authenticate/retry." >&2
    exit 1
  fi
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Install the system dependencies first: sudo bash scripts/install_system.sh" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
echo "Updating rosdep as the desktop user."
rosdep update --rosdistro humble
echo "Building workspace and camera environment."
bash scripts/bootstrap.sh --vision
source .venv/bin/activate
source install/setup.bash
python scripts/check_model.py
python -m unittest discover -s tests -q
for backend in fake gazebo_classic; do
  for mode in planned servo direct_ik; do
    python scripts/smoke_ros.py --backend "$backend" --mode "$mode"
  done
done
echo "OMX_F_BUILD_AND_SIMULATION_TESTS_COMPLETE"
if python scripts/check_camera.py --output artifacts/install/camera_report.json; then
  echo "Starting Gazebo, MoveIt, RViz, and hand tracking."
  exec bash scripts/run_hand_control.sh
else
  echo "Simulation tests passed; webcam not ready. Connect it and run bash scripts/run_hand_control.sh"
  exit 2
fi
