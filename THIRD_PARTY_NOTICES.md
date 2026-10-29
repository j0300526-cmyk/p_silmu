# Third-party material

ROBOTIS OpenMANIPULATOR-X, Copyright ROBOTIS CO., LTD., Apache License 2.0.
Source: https://github.com/ROBOTIS-GIT/open_manipulator
Pinned commit: 84b68b219a0fa30b223ad7e3c268d1f11fa43577 (humble).

- The upstream description is imported through dependencies.repos without modification.
- tests/fixtures/open_manipulator_x.urdf is an expanded geometry fixture derived from
  open_manipulator_x_description/urdf/open_manipulator_x.urdf.xacro (macro/prefix
  expansion and numeric expressions only).
- omx_moveit_config/config/omx.srdf adapts the named states and collision exclusions
  from open_manipulator_x_moveit_config/config/open_manipulator_x.srdf.
- omx_moveit_config/config/servo.yaml adapts the upstream moveit_servo.yaml, changing
  speed units, loop rate, input timeout, and planning scene monitor settings.
  The research extension also aligns its joint margin with the bridge configuration.

Research-inspired modules in input_processing.py and differential_ik.py are new
implementations of mathematical ideas, not copies of the cited papers' software.
See docs/research.md for original publications, scope, and departures from them.

The Apache 2.0 license is reproduced in LICENSE. MediaPipe/OpenCV/ROS/MoveIt are
separately installed dependencies and retain their respective licenses.

OMX-F geometry and eight mesh assets are derived from ROBOTIS open_manipulator
commit f60b8c4237f57bd832c0214ee2bb3f516b7402fb (Apache 2.0).
omx_f_arm.urdf.xacro changes package mesh URLs and joint-limit expressions.
omx_f.srdf changes the initial/home pose and end-effector/virtual-joint configuration.
tests/fixtures/omx_f.urdf is an expanded geometry fixture with simulation limits.
config/omx_f_assets.json records each original Git object; fetch_omx_f_assets.py
verifies downloaded mesh bytes before use. See docs/omx_ai.md for the port's scope.

omx_moveit_config/config/omx_f/joint_limits.yaml is an unchanged copy from the same
ROBOTIS commit. speed_reference.json records its SHA256 and source URL.
The local artifacts/reference snapshots are excluded from Git; installed dependencies
and mesh downloads are not redistributed as part of this repository.
