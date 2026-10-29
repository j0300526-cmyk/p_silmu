"""Cross-layer speed contracts against the pinned ROBOTIS MoveIt reference."""
import hashlib
import json
from pathlib import Path
import sys
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/omx_project/omx_teleop'))
from omx_teleop.differential_ik import DifferentialIK, DifferentialIKConfig
from omx_teleop.kinematics import load_arm_model


class OfficialSpeedTest(unittest.TestCase):
    def setUp(self):
        self.folder = ROOT/'src/omx_project/omx_moveit_config/config/omx_f'
        self.limits = yaml.safe_load((self.folder/'joint_limits.yaml').read_text())
        config = ROOT/'src/omx_project'
        self.bridge = yaml.safe_load((config/'omx_teleop/config/omx_f_official.yaml').read_text())['robot_bridge_node']['ros__parameters']
        self.hand = yaml.safe_load((config/'omx_hand_input/config/omx_f_official.yaml').read_text())['hand_control_node']['ros__parameters']

    def test_moveit_reference_is_unchanged_and_float_typed(self):
        manifest = json.loads((self.folder/'speed_reference.json').read_text())
        self.assertEqual(hashlib.sha256((self.folder/'joint_limits.yaml').read_bytes()).hexdigest(),
                         manifest['joint_limits_sha256'])
        for joint in self.limits['joint_limits'].values():
            self.assertIsInstance(joint['max_velocity'], float)
            self.assertIsInstance(joint['max_acceleration'], float)

    def test_servo_and_planned_share_official_effective_joint_limits(self):
        v = self.limits['default_velocity_scaling_factor']
        a = self.limits['default_acceleration_scaling_factor']
        self.assertEqual(self.bridge['velocity_scaling'], v)
        self.assertEqual(self.bridge['acceleration_scaling'], a)
        for name in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5'):
            limit = self.limits['joint_limits'][name]
            self.assertAlmostEqual(self.bridge['max_joint_velocity'], limit['max_velocity']*v)
            self.assertAlmostEqual(self.bridge['max_joint_acceleration'], limit['max_acceleration']*a)

    def test_input_and_pursuit_no_longer_impose_old_six_cm_cap(self):
        self.assertEqual(self.hand['max_speed'], self.bridge['max_cartesian_speed'])
        self.assertGreater(self.hand['max_speed'], .06)
        self.assertGreaterEqual(self.hand['max_target_lead']*self.bridge['servo_gain'],
                                self.bridge['max_cartesian_speed'])
        self.assertGreaterEqual(self.bridge['control_rate'], 20)

    def test_qp_obeys_official_scaled_limits_over_large_motion(self):
        model = load_arm_model((ROOT/'tests/fixtures/omx_f.urdf').read_text())
        p = self.bridge
        cfg = DifferentialIKConfig(gain=p['servo_gain'], max_cartesian_speed=p['max_cartesian_speed'],
                                   max_velocity=p['max_joint_velocity'], max_acceleration=p['max_joint_acceleration'])
        controller = DifferentialIK(model, cfg)
        q = [0, -1, .7, .3, 0]; previous = [0.0]*5
        goal = model.forward([.8, -1, .7, .3, 0])[:3]
        peak = 0.0
        for _ in range(400):
            result = controller.step(q, goal, previous, .02)
            self.assertTrue(result.feasible, result.reason)
            for value, old in zip(result.velocity, previous):
                self.assertLessEqual(abs(value), .5+1e-8)
                self.assertLessEqual(abs(value-old), .5*.02+1e-8)
            peak = max(peak, abs(result.velocity[0]))
            previous = result.velocity
            q = [value+.02*velocity for value, velocity in zip(q, previous)]
        self.assertGreater(peak, .4)
        self.assertAlmostEqual(q[0], .8, delta=.03)


if __name__ == '__main__':
    unittest.main()
