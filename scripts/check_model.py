#!/usr/bin/env python3
"""Expand both robot profiles and verify geometry, assets, and backend separation."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

from omx_teleop.kinematics import load_arm_model
from omx_teleop.profiles import get_robot_profile


def main():
    description = Path(get_package_share_directory('omx_description'))
    control = Path(get_package_share_directory('omx_control'))
    for name in ('omx_f', 'open_manipulator_x'):
        profile = get_robot_profile(name)
        for backend in ('fake', 'gazebo_classic'):
            filename = 'controllers_fake.yaml' if backend == 'fake' else 'controllers_sim.yaml'
            doc = xacro.process_file(str(description / 'urdf' / profile.urdf_file), mappings={
                'backend': backend,
                'controllers_file': str(control / 'config' / profile.control_directory / filename)})
            xml = doc.toxml()
            root = ET.fromstring(xml)
            model = load_arm_model(xml)
            assert model.joint_names == profile.joints
            assert len(root.findall('ros2_control')) == 1
            plugin = root.findtext('ros2_control/hardware/plugin')
            expected = ('mock_components/GenericSystem' if backend == 'fake'
                        else 'gazebo_ros2_control/GazeboSystem')
            assert plugin == expected, plugin
            gazebo_plugins = root.findall("gazebo/plugin[@filename='libgazebo_ros2_control.so']")
            assert len(gazebo_plugins) == (1 if backend == 'gazebo_classic' else 0)
            links = [link.get('name') for link in root.findall('link')]
            assert len(links) == len(set(links))
            for joint in root.findall('joint'):
                assert joint.find('parent').get('link') in links
                assert joint.find('child').get('link') in links
            initial = yaml.safe_load((description / 'config' / profile.initial_file).read_text())
            q = [initial[n] for n in profile.joints]
            point = model.forward(q)
            assert model.solve_position(point[:3], q, [0.0]) is not None
            for mesh in root.findall('.//mesh'):
                uri = mesh.get('filename')
                assert uri.startswith('package://')
                package, relative = uri[len('package://'):].split('/', 1)
                assert (Path(get_package_share_directory(package)) / relative).is_file(), uri
            gripper = next(j for j in root.findall('joint') if j.get('name') == profile.gripper_joint)
            lo, hi = (float(gripper.find('limit').get(key)) for key in ('lower', 'upper'))
            assert lo <= profile.gripper_close <= profile.gripper_open <= hi
            print('PASS:', name, backend, len(profile.joints), 'arm joints;', plugin)


if __name__ == '__main__':
    main()
