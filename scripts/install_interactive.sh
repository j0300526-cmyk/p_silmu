#!/usr/bin/env bash
# Open in a normal desktop terminal so Ubuntu can request the administrator password.
set -eo pipefail
cd "$(dirname "$0")/.."
echo "OMX-AI follower simulation: ROS 2 Humble / Gazebo / MoveIt installation"
echo "Enter your Ubuntu administrator password at the sudo prompt in THIS terminal."
mkdir -p artifacts/install
if sudo bash scripts/install_system.sh 2>&1 | tee artifacts/install/system_install.log; then
  echo "System packages installed. Returning to the ordinary desktop user for building."
  if ! bash scripts/finish_setup.sh 2>&1 | tee artifacts/install/workspace_setup.log; then
    echo "Inspect the logs in artifacts/install. Camera absence is reported separately."
  fi
else
  echo "System installation did not complete."
fi
read -r -p "Press Enter to close this installer terminal." _
