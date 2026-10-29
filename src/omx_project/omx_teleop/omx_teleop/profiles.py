"""Simulation profiles; these do not enable or initialize physical hardware."""
from dataclasses import dataclass


@dataclass(frozen=True)
class RobotProfile:
    name: str
    joints: tuple
    base_frame: str
    gripper_joint: str
    gripper_open: float
    gripper_close: float
    urdf_file: str
    srdf_file: str
    control_directory: str
    initial_file: str


def get_robot_profile(name):
    if name == 'omx_f':
        return RobotProfile(name, tuple('joint'+str(i) for i in range(1, 6)), 'link0',
                            'gripper_joint_1', 1.0, 0.0, 'omx_f_cell.urdf.xacro',
                            'omx_f.srdf', 'omx_f', 'omx_f_initial_positions.yaml')
    if name == 'open_manipulator_x':
        return RobotProfile(name, tuple('joint'+str(i) for i in range(1, 5)), 'link1',
                            'gripper_left_joint', .019, -.010, 'omx_cell.urdf.xacro',
                            'omx.srdf', '', 'initial_positions.yaml')
    raise ValueError('Unsupported robot profile: ' + str(name))
