#!/usr/bin/env bash
# One-time desktop resume for the user-authorized camera-fix reboot.
set -o pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
rm -f -- "${XDG_CONFIG_HOME:-$HOME/.config}/autostart/omx-camera-resume-once.desktop"
mkdir -p artifacts/install
echo 'Checking the built-in webcam, then starting Gazebo + MoveIt + hand control.'
echo 'Camera window: SPACE to start/stop, move one hand for arm, PINCH for gripper.'
bash scripts/run_hand_control.sh 2>&1 | tee artifacts/install/after_reboot.log
read -r -p 'Press Enter to close this terminal.' _
