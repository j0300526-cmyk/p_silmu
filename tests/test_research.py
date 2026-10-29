"""Algorithmic validation of research-inspired components; no ROS or NumPy."""
import math
from pathlib import Path
import random
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/omx_project/omx_teleop'))
from omx_teleop.differential_ik import (
    DifferentialIK, DifferentialIKConfig, box_qp, smallest_singular_value,
)
from omx_teleop.input_processing import (
    DwellHysteresis, OneEuroFilter, RelativeRetargeter, VectorFilter, hand_features,
)
from omx_teleop.kinematics import ArmGeometry

MODEL = ArmGeometry.from_urdf((ROOT / 'tests/fixtures/open_manipulator_x.urdf').read_text())


class FilterTest(unittest.TestCase):
    def test_low_pass_formula_and_step_adaptation(self):
        slow = OneEuroFilter(min_cutoff=1.0, beta=0.0)
        fast = OneEuroFilter(min_cutoff=1.0, beta=1.0)
        for filt in (slow, fast):
            self.assertEqual(filt.update(0.0, 0.0), 0.0)
        self.assertAlmostEqual(slow.update(1.0, .03), OneEuroFilter.alpha(1.0, .03))
        self.assertGreater(fast.update(1.0, .03), slow.filtered)

    def test_noise_reduction_irregular_sampling(self):
        rng = random.Random(20)
        filt = OneEuroFilter()
        raw, filtered, timestamp = [], [], 0.0
        for i in range(400):
            timestamp += rng.uniform(.02, .045)
            value = rng.gauss(0, .02)
            output = filt.update(value, timestamp)
            if i > 30:
                raw.append(value)
                filtered.append(output)
        self.assertLess(sum(v*v for v in filtered), .5 * sum(v*v for v in raw))

    def test_stale_gap_resets_and_bad_time_does_not_poison_state(self):
        filt = OneEuroFilter()
        filt.update(0.0, 1.0)
        with self.assertRaises(ValueError):
            filt.update(math.nan, 1.1)
        with self.assertRaises(ValueError):
            filt.update(2.0, .9)
        self.assertEqual(filt.update(3.0, 2.0), 3.0)

    def test_filter_selection_preserves_moving_average_baseline(self):
        filt = VectorFilter('moving_average', window=2)
        filt.update([0, 0, 0], 0)
        self.assertEqual(filt.update([2, 4, 6], .1), [1, 2, 3])
        filt.reset()
        self.assertEqual(filt.update([2, 4, 6], .2), [2, 4, 6])


class RetargetTest(unittest.TestCase):
    def test_dwell_rejects_single_frame_and_hysteresis_does_not_chatter(self):
        gesture = DwellHysteresis(dwell=.08)
        self.assertFalse(gesture.update(.1, 0))
        self.assertFalse(gesture.update(.5, .04))
        self.assertFalse(gesture.update(.1, .1))
        self.assertTrue(gesture.update(.1, .19))
        self.assertTrue(gesture.update(.26, .2))
        self.assertTrue(gesture.update(.5, .21))
        self.assertFalse(gesture.update(.5, .30))

    def test_relative_stationary_input_does_not_integrate_drift(self):
        mapper = RelativeRetargeter(calibration_frames=2)
        point = [.2, 0.0, .15]
        feature = [.3, .7, -.4]  # Deliberately away from the image centre.
        self.assertIsNone(mapper.update(feature, point, 0))
        self.assertEqual(mapper.update(feature, point, .03), point)
        for i in range(2, 400):
            self.assertEqual(mapper.update(feature, point, i*.03), point)

    def test_clutch_reengagement_and_precision_reindex_to_robot(self):
        mapper = RelativeRetargeter(calibration_frames=1)
        point = [.2, 0.0, .15]
        mapper.update([0, 0, 0], point, 0)
        mapper.update([0, .2, 0], point, .1)
        changed_robot = [.2, .001, .15]
        self.assertEqual(mapper.update([0, .2, 0], changed_robot, .2, resolution=.3), changed_robot)
        mapper.reset()
        self.assertEqual(mapper.update([2, 3, 4], changed_robot, .3), changed_robot)

    def test_target_speed_lead_and_workspace_limits(self):
        mapper = RelativeRetargeter(calibration_frames=1, max_speed=.05, max_lead=.02)
        robot = [.2, 0.0, .15]
        prior = mapper.update([0, 0, 0], robot, 0)
        for i in range(1, 101):
            target = mapper.update([10, 10, 10], robot, i * .02)
            self.assertLessEqual(math.dist(prior, target), .05*.02 + 1e-9)
            self.assertLessEqual(math.dist(robot, target), .02 + 1e-9)
            for v, lo, hi in zip(target, mapper.lower, mapper.upper):
                self.assertTrue(lo <= v <= hi)
            prior = target

    def test_feedback_jump_requires_recalibration(self):
        mapper = RelativeRetargeter(calibration_frames=1)
        mapper.update([0, 0, 0], [.2, 0, .15], 0)
        self.assertIsNone(mapper.update([0, 0, 0], [.2, .2, .15], .03))
        self.assertFalse(mapper.calibrated)

    def test_unstable_neutral_observation_is_not_calibrated(self):
        mapper = RelativeRetargeter(calibration_frames=3, calibration_tolerance=.01)
        for i in range(8):
            self.assertIsNone(mapper.update([i*.1, 0, 0], [.2, 0, .15], i*.03))

    def test_landmark_resolution_invariance_and_invalid_palm(self):
        points = [(320.0, 240.0)] * 21
        points[9], points[4], points[8] = (320, 340), (300, 200), (310, 200)
        features, pinch = hand_features(points, 640, 480)
        scaled, scaled_pinch = hand_features([(2*x, 2*y) for x, y in points], 1280, 960)
        self.assertEqual(features, scaled)
        self.assertEqual(pinch, scaled_pinch)
        with self.assertRaises(ValueError):
            hand_features([(0, 0)] * 21, 640, 480)


class QPTest(unittest.TestCase):
    def test_coupled_box_solution_is_not_posthoc_clipping(self):
        # Unconstrained optimum is (2,-1); clipping would give (1,-1).
        h = [[2.0, 1.0], [1.0, 2.0]]
        self.assertEqual(box_qp(h, [-3, 0], [-1, -1], [1, 1]), [1, -.5])

    def test_100_random_spd_problems_satisfy_kkt(self):
        rng = random.Random(3)
        for _ in range(100):
            a = [[rng.uniform(-1, 1) for _ in range(4)] for _ in range(4)]
            h = [[sum(row[i]*row[j] for row in a) + (.1 if i == j else 0)
                  for j in range(4)] for i in range(4)]
            g = [rng.uniform(-3, 3) for _ in range(4)]
            lo = [rng.uniform(-.8, -.1) for _ in range(4)]
            hi = [rng.uniform(.1, .8) for _ in range(4)]
            x = box_qp(h, g, lo, hi)
            for i, value in enumerate(x):
                derivative = sum(h[i][j] * x[j] for j in range(4)) + g[i]
                self.assertTrue(lo[i] - 1e-9 <= value <= hi[i] + 1e-9)
                if abs(value - lo[i]) < 1e-8:
                    self.assertGreaterEqual(derivative, -1e-7)
                elif abs(value - hi[i]) < 1e-8:
                    self.assertLessEqual(derivative, 1e-7)
                else:
                    self.assertAlmostEqual(derivative, 0, places=7)

    def test_invalid_problem_and_infeasible_bounds(self):
        self.assertIsNone(box_qp([[1]], [0], [1], [-1]))
        with self.assertRaises(ValueError):
            box_qp([[-1]], [0], [-1], [1])
        with self.assertRaises(ValueError):
            box_qp([[math.nan]], [0], [-1], [1])

    def test_singular_values(self):
        self.assertAlmostEqual(smallest_singular_value(
            [[2, 0, 0, 0], [0, .5, 0, 0], [0, 0, .2, 0]]), .2)
        c, s = math.cos(.7), math.sin(.7)
        self.assertAlmostEqual(smallest_singular_value(
            [[2*c, -.5*s, 0, 0], [2*s, .5*c, 0, 0], [0, 0, .2, 0]]), .2)
        self.assertAlmostEqual(smallest_singular_value(
            [[1, 0, 0, 0], [1, 0, 0, 0], [0, 1, 0, 0]]), 0, places=7)


class DifferentialIKTest(unittest.TestCase):
    def test_jacobian_matches_finite_difference(self):
        rng = random.Random(4)
        for _ in range(100):
            q = [rng.uniform(lo + .1, hi - .1) for lo, hi in MODEL.limits]
            jacobian = MODEL.position_jacobian(q)
            for j in range(4):
                plus, minus = list(q), list(q)
                plus[j] += 1e-6
                minus[j] -= 1e-6
                numeric = [(a-b)/2e-6 for a, b in zip(MODEL.forward(plus)[:3], MODEL.forward(minus)[:3])]
                for i in range(3):
                    self.assertAlmostEqual(jacobian[i][j], numeric[i], places=8)

    def test_joint_margin_velocity_and_acceleration_over_closed_loop(self):
        controller = DifferentialIK(MODEL)
        q, previous = [0, -1, .7, .3], [0.0]*4
        target = MODEL.forward([.25, -.8, .5, .3])[:3]
        initial_error = math.dist(MODEL.forward(q)[:3], target)
        for _ in range(300):
            result = controller.step(q, target, previous, .05)
            self.assertTrue(result.feasible, result.reason)
            for value, old in zip(result.velocity, previous):
                self.assertLessEqual(abs(value), .4 + 1e-9)
                self.assertLessEqual(abs(value - old), .8*.05 + 1e-9)
            q = [v + .05*d for v, d in zip(q, result.velocity)]
            for value, (lo, hi) in zip(q, MODEL.limits):
                self.assertTrue(lo + .08 - 1e-9 <= value <= hi - .08 + 1e-9)
            previous = result.velocity
        self.assertLess(math.dist(MODEL.forward(q)[:3], target), initial_error * .1)

    def test_outward_command_slows_before_joint_margin(self):
        controller = DifferentialIK(MODEL)
        q = [MODEL.limits[0][1] - .09, -1, .7, .3]
        current = MODEL.forward(q)[:3]
        j = MODEL.position_jacobian(q)
        target = [p + row[0]*2 for p, row in zip(current, j)]
        result = controller.step(q, target, [0.0]*4, .05)
        self.assertTrue(result.feasible)
        self.assertLessEqual(result.velocity[0], .02 + 1e-9)

    def test_conflicting_acceleration_and_barrier_fail_closed(self):
        controller = DifferentialIK(MODEL)
        q = [MODEL.limits[0][1] - .0801, -1, .7, .3]
        result = controller.step(q, MODEL.forward(q)[:3], [.4, 0, 0, 0], .05)
        self.assertFalse(result.feasible)
        self.assertEqual(result.velocity, [0.0]*4)

    def test_overrun_nonfinite_and_singularity_remain_bounded(self):
        controller = DifferentialIK(MODEL)
        for dt in (0.0, .2, math.nan):
            self.assertFalse(controller.step([0, -1, .7, .3], [.2, 0, .2], [0]*4, dt).feasible)
        # Fully extended shoulder-to-tool chain is translationally rank deficient.
        delta = math.atan2(MODEL.upper_x, MODEL.upper_z)
        q = [0, math.pi/2-delta, -math.pi/2+delta, 0]
        result = controller.step(q, [1.0, 0, .15], [0]*4, .05)
        self.assertTrue(result.feasible)
        self.assertLess(result.sigma_min, 1e-6)
        self.assertTrue(all(math.isfinite(v) and abs(v) <= .4 for v in result.velocity))
        self.assertAlmostEqual(result.damping, .06, places=7)


if __name__ == '__main__':
    unittest.main()
