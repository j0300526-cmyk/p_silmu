# Source only within this project's launch scripts.
unset GSETTINGS_SCHEMA_DIR GTK_PATH GIO_EXTRA_MODULES GI_TYPELIB_PATH
export XDG_DATA_DIRS=/usr/local/share:/usr/share
omx_workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$omx_workspace_root"
if [[ ! -f /opt/ros/humble/setup.bash || ! -f install/setup.bash || ! -x .venv/bin/python ]]; then
  echo "Install ROS and build with bash scripts/bootstrap.sh --vision first." >&2
  return 1
fi
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
