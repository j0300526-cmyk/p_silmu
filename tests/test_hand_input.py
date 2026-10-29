"""One-hand selection and camera callback checks; no webcam or robot commands."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
for package in ('omx_teleop', 'omx_hand_input'):
    sys.path.insert(0, str(ROOT / 'src/omx_project' / package))
from omx_teleop.input_processing import (
    ControlHandSelector, DwellHysteresis, RelativeRetargeter, VectorFilter,
)
try:
    from omx_hand_input.hand_control_node import HandControlNode
except ModuleNotFoundError as exc:
    if exc.name not in ('rclpy', 'geometry_msgs', 'rcl_interfaces', 'sensor_msgs', 'std_msgs', 'std_srvs'):
        raise
    HandControlNode = None


class HandSelectionTest(unittest.TestCase):
    def test_either_single_hand_can_control(self):
        for label in ('Left', 'Right'):
            selector = ControlHandSelector()
            self.assertEqual(selector.update({label}, True), label)

    def test_other_hand_cannot_take_over_when_selected_hand_disappears(self):
        selector = ControlHandSelector()
        self.assertEqual(selector.update({'Right'}, True), 'Right')
        self.assertEqual(selector.update({'Left', 'Right'}, True), 'Right')
        self.assertIsNone(selector.update({'Left'}, True))
        self.assertEqual(selector.selected, 'Right')
        selector.update({'Left'}, False)
        self.assertEqual(selector.update({'Left'}, True), 'Left')

    def test_explicit_preference_does_not_fall_back_to_other_hand(self):
        selector = ControlHandSelector('left')
        self.assertIsNone(selector.update({'Right'}, True))
        self.assertEqual(selector.update({'Left', 'Right'}, True), 'Left')
        with self.assertRaises(ValueError):
            ControlHandSelector('unknown')


class Sink:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@unittest.skipIf(HandControlNode is None, 'ROS message packages needed for callback checks')
class HandCallbackTest(unittest.TestCase):
    def setUp(self):
        # Exercise the real callback with deterministic landmarks and in-memory sinks.
        # Do not initialize a ROS graph or open a second camera beside the live stack.
        n = self.node = object.__new__(HandControlNode)
        parameters = dict(hand_scheme='one_hand', control_hand='auto', show_window=False,
                          deadzone_pixels=40.0, mapping_mode='relative', input_filter='one_euro',
                          clutch_enabled=True, precision_scale=.3)
        n.p = parameters.__getitem__
        self.parameters = parameters
        n.hand_selector = ControlHandSelector()
        n.selected_hand, n.detected_hands = None, []
        n.armed = n.tracking = n.quit = n.precision = False
        n.current_q, n.target = [0.0] * 5, None
        n.last_frame, n.feedback_at = 0.0, 0.0
        n.status_text, n.gripper_state, n.target_frame = 'DISARMED', 'RELEASE', 'link0'
        n.mapper = RelativeRetargeter(calibration_frames=2)
        n.input_filter = VectorFilter()
        n.clutch, n.grasp = DwellHysteresis(), DwellHysteresis()
        for name in ('pose_pub', 'gripper_pub', 'enable_pub', 'status_pub'):
            setattr(n, name, Sink())
        n.model = NS(forward=lambda q: [.2, 0.0, .15], solve_position=lambda *args: [0.0]*5)
        n.cap = NS(read=lambda: (True, NS(shape=(480, 640, 3))))
        n.cv2 = NS(flip=lambda f, axis: f, cvtColor=lambda f, kind: f, COLOR_BGR2RGB=1)
        self.results = NS(multi_hand_landmarks=[], multi_handedness=[])
        n.hands = NS(process=lambda f: self.results)
        n.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: __import__(
            'builtin_interfaces.msg', fromlist=['Time']).Time(sec=1)))
        n.get_logger = lambda: NS(warn=lambda *args, **kwargs: None)
        n.set_armed(True)

    def frame(self, time, hands, fresh=True):
        landmarks, handedness = [], []
        for label, closed, x in hands:
            points = [NS(x=x, y=.5) for _ in range(21)]
            points[9] = NS(x=x, y=.7)
            points[4] = NS(x=x-.1, y=.45)
            points[8] = NS(x=x-.09 if closed else x+.12, y=.45)
            landmarks.append(NS(landmark=points))
            handedness.append(NS(classification=[NS(label=label)]))
        self.results.multi_hand_landmarks = landmarks
        self.results.multi_handedness = handedness
        if fresh:
            self.node.feedback_at = time
        with patch('omx_hand_input.hand_control_node.time.monotonic', return_value=time):
            self.node.process_frame()

    def test_single_right_hand_moves_arm_and_closes_then_opens_gripper(self):
        for t, closed in [(10, False), (10.05, False), (10.1, True), (10.2, True),
                          (10.25, False), (10.35, False)]:
            self.frame(t, [('Right', closed, .5)])
        n = self.node
        self.assertTrue(n.tracking)
        self.assertEqual(n.selected_hand, 'Right')
        commands = [m.data for m in n.gripper_pub.messages]
        self.assertIn('GRASP', commands)
        self.assertEqual(commands[-1], 'RELEASE')
        # Finger closure alone must not change the Cartesian arm target.
        for msg in n.pose_pub.messages:
            self.assertEqual([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], [.2, 0, .15])
        self.frame(10.4, [('Right', False, .52)])
        self.assertGreater(n.pose_pub.messages[-1].pose.position.y, 0)

    def test_selected_hand_loss_with_other_hand_visible_stops_and_requires_rearm(self):
        self.frame(10, [('Right', False, .5)])
        self.frame(10.05, [('Right', False, .5)])
        count = len(self.node.pose_pub.messages)
        self.frame(10.1, [('Left', True, .7)])
        self.frame(10.15, [('Left', True, .7)])
        self.assertFalse(self.node.armed)
        self.assertFalse(self.node.enable_pub.messages[-1].data)
        self.assertEqual(len(self.node.pose_pub.messages), count)
        self.node.set_armed(True)
        self.frame(10.2, [('Left', False, .7)])
        self.frame(10.25, [('Left', False, .7)])
        self.assertTrue(self.node.tracking)
        self.assertEqual(self.node.selected_hand, 'Left')
        self.assertEqual(self.node.pose_pub.messages[-1].pose.position.y, 0)

    def test_stale_feedback_disarms_even_with_visible_hand(self):
        self.frame(10, [('Right', False, .5)])
        self.frame(10.05, [('Right', False, .5)])
        self.frame(11, [('Right', False, .5)], fresh=False)
        self.assertFalse(self.node.armed)
        self.assertFalse(self.node.enable_pub.messages[-1].data)

    def test_ambiguous_handedness_does_not_transfer_control(self):
        self.frame(10, [('Right', False, .5)])
        self.frame(10.05, [('Right', False, .5)])
        self.frame(10.1, [('Right', False, .5), ('Right', True, .7)])
        self.assertFalse(self.node.armed)

    def test_two_hand_option_retains_left_pinch_clutch(self):
        self.parameters['hand_scheme'] = 'two_hand'
        self.frame(10, [('Left', False, .5), ('Right', True, .7)])
        self.assertFalse(self.node.tracking)
        self.assertFalse(self.node.pose_pub.messages)
        for t in [10.05, 10.15, 10.2, 10.25, 10.35]:
            self.frame(t, [('Left', True, .5), ('Right', True, .7)])
        self.assertTrue(self.node.tracking)
        self.assertIn('GRASP', [m.data for m in self.node.gripper_pub.messages])


if __name__ == '__main__':
    unittest.main()
