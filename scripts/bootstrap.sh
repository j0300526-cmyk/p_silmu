#!/usr/bin/env bash
# Run on Ubuntu 22.04 with ROS 2 Humble already installed.
set -eo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
vision=false
if [[ "${1:-}" == "--vision" ]]; then
  vision=true
elif [[ -n "${1:-}" ]]; then
  echo "Usage: bash scripts/bootstrap.sh [--vision]" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is required. Follow README.md on Ubuntu 22.04." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
for tool in vcs rosdep colcon; do
  command -v "$tool" >/dev/null || { echo "Missing $tool; see README.md" >&2; exit 1; }
done
colcon_cli="$(command -v colcon)"
mkdir -p src/vendor
python3 scripts/fetch_omx_f_assets.py
vcs import --skip-existing src/vendor < dependencies.repos
actual_commit="$(git -C src/vendor/open_manipulator rev-parse HEAD)"
if [[ "$actual_commit" != "84b68b219a0fa30b223ad7e3c268d1f11fa43577" ]]; then
  echo "Vendor checkout differs from dependencies.repos. Resolve it before building." >&2
  exit 1
fi
rosdep install --from-paths src/omx_project \
  src/vendor/open_manipulator/open_manipulator_x_description \
  --ignore-src --rosdistro humble -r -y
if $vision; then
  python3 -m venv --system-site-packages .venv
  source .venv/bin/activate
  python3 -m pip install -r requirements-vision.txt
fi
python3 "$colcon_cli" build --symlink-install --packages-up-to omx_bringup \
  --event-handlers console_direct+
echo "Build finished. Source /opt/ros/humble/setup.bash and install/setup.bash."
if $vision; then
  echo "Activate .venv/bin/activate in camera terminals as well."
fi
