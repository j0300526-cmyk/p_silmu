"""OMX-F five-axis checks independent of the OpenManipulator-X analytic model."""
import hashlib
import json
import math
from pathlib import Path
import random
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/omx_project/omx_teleop'))
from omx_teleop.differential_ik import DifferentialIK, box_qp
from omx_teleop.kinematics import load_arm_model
from omx_teleop.profiles import get_robot_profile

XML = (ROOT / 'tests/fixtures/omx_f.urdf').read_text()
MODEL = load_arm_model(XML)


def matmul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def matrix_fk(q):
    joints = {j.get('name'): j for j in ET.fromstring(XML).findall('joint')}
    matrix = [[float(i == j) for j in range(4)] for i in range(4)]
    for name, angle in zip((*MODEL.joint_names, 'end_effector_joint'), (*q, 0.0)):
        joint = joints[name]
        translation = [[float(i == j) for j in range(4)] for i in range(4)]
        for i, value in enumerate(joint.find('origin').get('xyz').split()):
            translation[i][3] = float(value)
        axis = joint.find('axis')
        rotation = [[float(i == j) for j in range(4)] for i in range(4)]
        if axis is not None:
            x, y, z = [float(v) for v in axis.get('xyz').split()]
            c, s, v = math.cos(angle), math.sin(angle), 1-math.cos(angle)
            block = [[c+x*x*v, x*y*v-z*s, x*z*v+y*s],
                     [y*x*v+z*s, c+y*y*v, y*z*v-x*s],
                     [z*x*v-y*s, z*y*v+x*s, c+z*z*v]]
            for i in range(3):
                for j in range(3):
                    rotation[i][j] = block[i][j]
        matrix = matmul(matmul(matrix, translation), rotation)
    return [matrix[i][3] for i in range(3)]


class OMXFTest(unittest.TestCase):
    def test_distinct_profiles_and_frames(self):
        follower = get_robot_profile('omx_f')
        legacy = get_robot_profile('open_manipulator_x')
        self.assertEqual(follower.joints, MODEL.joint_names)
        self.assertEqual(len(follower.joints), 5)
        self.assertEqual(len(legacy.joints), 4)
        self.assertEqual(follower.base_frame, 'link0')
        self.assertEqual(legacy.base_frame, 'link1')
        self.assertEqual(follower.gripper_joint, 'gripper_joint_1')

    def test_300_fk_and_ik_cases_with_roll_and_base_windings(self):
        rng = random.Random(51)
        for _ in range(300):
            q = [rng.uniform(lo+.1, hi-.1) for lo, hi in MODEL.limits]
            xyz = matrix_fk(q)
            self.assertLess(math.dist(MODEL.forward(q)[:3], xyz), 1e-10)
            result = MODEL.solve_position(xyz, q, [0.0, .4, .8])
            self.assertIsNotNone(result, q)
            self.assertLess(math.dist(result, q), 1e-7)

    def test_five_column_jacobian_matches_independent_fk(self):
        rng = random.Random(52)
        for _ in range(80):
            q = [rng.uniform(lo+.1, hi-.1) for lo, hi in MODEL.limits]
            j = MODEL.position_jacobian(q)
            for column in range(5):
                a, b = list(q), list(q)
                a[column] += 1e-6
                b[column] -= 1e-6
                numeric = [(x-y)/2e-6 for x, y in zip(matrix_fk(a), matrix_fk(b))]
                for row in range(3):
                    self.assertAlmostEqual(j[row][column], numeric[row], places=8)

    def test_five_axis_qp_closed_loop(self):
        controller = DifferentialIK(MODEL)
        q, velocity = [0.0, -1.0, .7, .3, 0.0], [0.0]*5
        target = MODEL.forward([.2, -.8, .6, .2, .1])[:3]
        initial_error = math.dist(MODEL.forward(q)[:3], target)
        for _ in range(300):
            result = controller.step(q, target, velocity, .05)
            self.assertTrue(result.feasible, result.reason)
            self.assertEqual(len(result.velocity), 5)
            for value, prior in zip(result.velocity, velocity):
                self.assertLessEqual(abs(value), .4+1e-9)
                self.assertLessEqual(abs(value-prior), .8*.05+1e-9)
            velocity = result.velocity
            q = [value+.05*speed for value, speed in zip(q, velocity)]
            for value, (lo, hi) in zip(q, MODEL.limits):
                self.assertTrue(lo+.08-1e-9 <= value <= hi-.08+1e-9)
        self.assertLess(math.dist(MODEL.forward(q)[:3], target), initial_error*.1)

    def test_five_variable_qp_active_constraints(self):
        h = [[2.0 if i == j else .1 for j in range(5)] for i in range(5)]
        g = [-3, 1, 4, -2, 0]
        x = box_qp(h, g, [-.2]*5, [.2]*5)
        self.assertEqual(len(x), 5)
        for i, value in enumerate(x):
            derivative = g[i] + sum(h[i][j]*x[j] for j in range(5))
            self.assertTrue(-.2-1e-9 <= value <= .2+1e-9)
            if abs(value+.2) < 1e-8:
                self.assertGreaterEqual(derivative, -1e-8)
            elif abs(value-.2) < 1e-8:
                self.assertLessEqual(derivative, 1e-8)
            else:
                self.assertAlmostEqual(derivative, 0, places=8)

    def test_rotary_gripper_and_opposite_mimic(self):
        joints = {j.get('name'): j for j in ET.fromstring(XML).findall('joint')}
        primary, secondary = joints['gripper_joint_1'], joints['gripper_joint_2']
        self.assertEqual(primary.get('type'), 'revolute')
        self.assertEqual(secondary.find('mimic').get('multiplier'), '-1')
        profile = get_robot_profile('omx_f')
        self.assertEqual(profile.gripper_close, 0.0)
        self.assertEqual(profile.gripper_open, 1.0)

    def test_pinned_meshes_match_upstream_objects(self):
        folder = ROOT / 'src/omx_project/omx_description'
        manifest = json.loads((folder / 'config/omx_f_assets.json').read_text())
        self.assertEqual(len(manifest['assets']), 8)
        for asset in manifest['assets']:
            path = folder / asset['destination']
            if not path.exists():
                self.skipTest('Run python3 scripts/fetch_omx_f_assets.py to obtain mesh assets')
            data = path.read_bytes()
            sha = hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
            self.assertEqual(sha, asset['git_blob_sha1'])

    def test_unknown_robot_does_not_silently_use_four_axis_model(self):
        with self.assertRaises(ValueError):
            load_arm_model(XML.replace('name="omx_f"', 'name="unknown_robot"', 1))


if __name__ == '__main__':
    unittest.main()
