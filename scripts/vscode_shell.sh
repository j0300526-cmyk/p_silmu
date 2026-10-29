# VS Code workspace terminal initialization (sourced by interactive bash).
if [[ -f "$HOME/.bashrc" ]]; then
  source "$HOME/.bashrc"
fi
if [[ "${GSETTINGS_SCHEMA_DIR:-}" == *"/snap/code/"* ]]; then
  unset GSETTINGS_SCHEMA_DIR GTK_PATH GIO_EXTRA_MODULES GI_TYPELIB_PATH
fi
omx_workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$omx_workspace_root/.venv/bin/activate" ]]; then
  source "$omx_workspace_root/.venv/bin/activate"
fi
if [[ -f "$omx_workspace_root/install/setup.bash" ]]; then
  source "$omx_workspace_root/install/setup.bash"
fi
