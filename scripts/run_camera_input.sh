#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/runtime_environment.sh"
if [[ "${1:-}" == '--wait' ]]; then
  shift
  echo 'Waiting for a readable /dev/video* webcam. Ctrl+C stops waiting.'
  python scripts/check_camera.py --output artifacts/install/camera_report.json || true
  until compgen -G '/dev/video*' >/dev/null && \
      python scripts/check_camera.py --output artifacts/install/camera_report.json; do
    sleep 3
  done
else
  python scripts/check_camera.py --output artifacts/install/camera_report.json
fi
# Join the selected simulation profile rather than silently using old input caps.
omx_speed_profile=$(python - <<'PY'
import rclpy
from rcl_interfaces.srv import GetParameters
rclpy.init()
node = rclpy.create_node('omx_camera_profile_probe')
client = node.create_client(GetParameters, '/robot_bridge_node/get_parameters')
if not client.wait_for_service(timeout_sec=5.0):
    raise SystemExit('Start the simulation before attaching the camera.')
request = GetParameters.Request(); request.names = ['speed_profile']
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
if not future.done() or future.result() is None:
    raise SystemExit('Could not read the running speed profile.')
print(future.result().values[0].string_value)
node.destroy_node(); rclpy.shutdown()
PY
)
omx_camera_profile_args=()
if [[ "$omx_speed_profile" == omx_f_official_default ]]; then
  omx_camera_profile_args=(--params-file src/omx_project/omx_hand_input/config/omx_f_official.yaml)
fi
exec ros2 run omx_hand_input hand_control_node --ros-args \
  --params-file src/omx_project/omx_hand_input/config/hand_control.yaml \
  "${omx_camera_profile_args[@]}" \
  -p use_sim_time:=true "$@"
