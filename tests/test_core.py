"""Offline tests: python3 -m unittest discover -s tests -v."""
import math
from pathlib import Path
import random
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src/omx_project' / p) for p in ('omx_teleop', 'omx_bringup')]
from omx_teleop.kinematics import ArmGeometry, JOINTS
from omx_teleop.safety import InputGate, limited_step, smooth_duration, stamp_is_fresh
from omx_bringup.scene import load_objects, world_xml

FIXTURE = (ROOT / 'tests/fixtures/open_manipulator_x.urdf').read_text()


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def urdf_fk(q):
    """Independent homogeneous-transform FK; no analytical model constants."""
    root = ET.fromstring(FIXTURE)
    joints = {j.get('name'): j for j in root.findall('joint')}
    matrix = [[float(i == j) for j in range(4)] for i in range(4)]
    for name, angle in zip((*JOINTS, 'end_effector_joint'), (*q, 0.0)):
        joint = joints[name]
        translation = [[float(i == j) for j in range(4)] for i in range(4)]
        for i, value in enumerate(joint.find('origin').get('xyz').split()):
            translation[i][3] = float(value)
        rotation = [[float(i == j) for j in range(4)] for i in range(4)]
        c, s = math.cos(angle), math.sin(angle)
        axis = joint.find('axis')
        if axis is not None and axis.get('xyz') == '0 0 1':
            rotation[0][:2], rotation[1][:2] = [c, -s], [s, c]
        elif axis is not None:
            rotation[0][0], rotation[0][2] = c, s
            rotation[2][0], rotation[2][2] = -s, c
        matrix = multiply(multiply(matrix, translation), rotation)
    return [matrix[i][3] for i in range(3)]


class KinematicsTest(unittest.TestCase):
    def setUp(self):
        self.model = ArmGeometry.from_urdf(FIXTURE)

    def test_pinned_urdf_offsets_and_limits(self):
        self.assertAlmostEqual(self.model.base_x, 0.012)
        self.assertAlmostEqual(self.model.shoulder_z, 0.0765)
        self.assertEqual(self.model.limits[1], (-1.5, 1.5))
        self.assertEqual(self.model.limits[2], (-1.5, 1.4))
        self.assertEqual(self.model.limits[3], (-1.7, 1.97))
        self.assertAlmostEqual(self.model.forward([0, 0, 0, 0])[0], 0.286)
        self.assertAlmostEqual(self.model.forward([0, 0, 0, 0])[2], 0.2045)

    def test_500_poses_against_independent_urdf_fk_and_ik(self):
        rng = random.Random(103)
        for _ in range(500):
            q = [rng.uniform(lo + .02, hi - .02) for lo, hi in self.model.limits]
            expected = urdf_fk(q)
            self.assertLess(math.dist(self.model.forward(q)[:3], expected), 1e-10)
            candidates = self.model.solutions(*expected, sum(q[1:]))
            self.assertTrue(candidates, q)
            self.assertLess(min(math.dist(c, q) for c in candidates), 1e-7)
            for candidate in candidates:
                self.assertLess(math.dist(urdf_fk(candidate), expected), 1e-7)

    def test_current_configuration_is_preserved(self):
        q = [0.8, -0.6, 0.3, 0.5]
        target = urdf_fk(q)
        result = self.model.solve_position(target, q, [0.0, .4, .8, -.4, 1.2])
        self.assertLess(math.dist(q, result), 1e-9)

    def test_nonfinite_and_unreachable_targets(self):
        for xyz in [(3, 0, 0), (0, math.nan, .1), (math.inf, 0, 0)]:
            self.assertEqual(self.model.solutions(*xyz, 0), [])

    def test_unsupported_model_fails_closed(self):
        with self.assertRaises(ValueError):
            ArmGeometry.from_urdf(FIXTURE.replace('xyz="0 1 0"', 'xyz="1 0 0"'))


class SafetyTest(unittest.TestCase):
    def ready_gate(self):
        gate = InputGate()
        gate.set_enabled(True, 10.0)
        gate.pose_at = gate.joints_at = 10.0
        return gate

    def test_input_and_feedback_timeouts(self):
        gate = self.ready_gate()
        self.assertTrue(gate.ready(10.1))
        self.assertFalse(gate.ready(10.4))
        gate.enable_at = gate.pose_at = 10.6
        self.assertFalse(gate.ready(10.6))
        gate.joints_at = 10.6
        self.assertTrue(gate.ready(10.6))

    def test_explicit_stop_requires_disable_then_enable(self):
        gate = self.ready_gate()
        gate.stop()
        gate.set_enabled(True, 10.1)
        self.assertFalse(gate.ready(10.1))
        gate.set_enabled(False, 10.1)
        self.assertFalse(gate.ready(10.1))
        gate.set_enabled(True, 10.2)
        self.assertTrue(gate.ready(10.2))

    def test_stamp_does_not_accept_old_or_future_time(self):
        self.assertTrue(stamp_is_fresh(10.0, 10.1, .35))
        self.assertFalse(stamp_is_fresh(10.0, 11.0, .35))
        self.assertFalse(stamp_is_fresh(11.0, 10.0, .35))
        self.assertFalse(stamp_is_fresh(math.nan, 10.0, .35))

    def test_step_and_quintic_velocity_acceleration_bounds(self):
        target = limited_step([0, 0, 0, 0], [2, -1, 0, .2], .08)
        self.assertAlmostEqual(target[0], .08)
        self.assertAlmostEqual(target[1], -.04)
        duration = smooth_duration([0] * 4, target, .3, .4, .8)
        for tick in range(1001):
            u = tick / 1000
            velocity = abs(.08 * (30*u*u - 60*u**3 + 30*u**4) / duration)
            accel = abs(.08 * (60*u - 180*u*u + 120*u**3) / duration**2)
            self.assertLessEqual(velocity, .400001)
            self.assertLessEqual(accel, .8001)


class SceneTest(unittest.TestCase):
    def test_gazebo_geometry_matches_shared_scene(self):
        objects = load_objects(ROOT / 'src/omx_project/omx_simulation/config/workcell_objects.yaml')
        world = ET.fromstring(world_xml(objects)).find('world')
        for item, model in zip(objects, world.findall('model')):
            self.assertEqual(item['id'], model.get('name'))
            self.assertEqual(item['position'], [float(v) for v in model.findtext('pose').split()[:3]])
            self.assertEqual(item['size'], [
                float(v) for v in model.findtext('link/collision/geometry/box/size').split()])

    def test_invalid_and_duplicate_obstacles_rejected(self):
        for source in (
            'objects: [{id: x, size: [-1, 1, 1], position: [0, 0, 0]}]',
            'objects: [{id: x, size: [1, 1, 1], position: [0, 0, 0]},'
            '{id: x, size: [1, 1, 1], position: [0, 0, 0]}]',
        ):
            with tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp) / 'scene.yaml'
                path.write_text(source)
                with self.assertRaises(ValueError):
                    load_objects(path)


if __name__ == '__main__':
    unittest.main()
