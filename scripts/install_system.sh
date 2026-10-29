#!/usr/bin/env bash
# System-only installer; call via pkexec or sudo. Workspace build runs as the user.
set -eo pipefail
if [[ "$(id -u)" != 0 ]]; then
  echo "Run with: sudo bash scripts/install_system.sh" >&2
  exit 1
fi
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 || "$(dpkg --print-architecture)" != amd64 ]]; then
  echo "This installer targets Ubuntu 22.04 amd64." >&2
  exit 1
fi
export DEBIAN_FRONTEND=noninteractive
export LC_ALL=C.UTF-8
task_tmp="$(mktemp -d /tmp/omx-system-install.XXXXXX)"
trap 'rm -rf -- "$task_tmp"' EXIT
apt-get -o DPkg::Lock::Timeout=120 update
apt-get -o DPkg::Lock::Timeout=120 install -y curl ca-certificates software-properties-common
add-apt-repository -y universe
source_version=1.3.0
source_sha256=110b9a462d55252decb8b7c816f61c2ba0d9890ce5fb93ac504e97cae5860d76
source_deb="$task_tmp/ros2-apt-source.deb"
curl --fail --location --silent --show-error --retry 3 --connect-timeout 15 \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$source_version/ros2-apt-source_$source_version.jammy_all.deb" \
  --output "$source_deb"
echo "$source_sha256  $source_deb" | sha256sum --check
dpkg -i "$source_deb"
apt-get -o DPkg::Lock::Timeout=120 update
packages=(
  ros-humble-desktop ros-humble-moveit ros-humble-moveit-servo
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gripper-controllers
  ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-tf2-ros-py
  ros-humble-rmw-cyclonedds-cpp
  python3-colcon-common-extensions python3-vcstool python3-rosdep
  python3-venv python3-pip python3-setuptools python3-yaml python3-opencv
  build-essential cmake git v4l-utils mesa-utils xvfb libportaudio2
)
apt-get -s install "${packages[@]}" > "$task_tmp/apt-plan.txt"
if grep -q '^Remv ' "$task_tmp/apt-plan.txt"; then
  cat "$task_tmp/apt-plan.txt"
  echo "Installation would remove existing packages; stopped before making that change." >&2
  exit 1
fi
apt-get -o DPkg::Lock::Timeout=120 install -y "${packages[@]}"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
echo "OMX_SYSTEM_INSTALL_COMPLETE"
