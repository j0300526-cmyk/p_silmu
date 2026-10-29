"""Exercise actual asynchronous action-slot transitions without ROS dependencies."""
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/omx_project/omx_teleop'))
from omx_teleop.action_slot import ActionSlot


class Client:
    def __init__(self):
        self.future = Future()

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        return self.future


class Handle:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancel_count = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancel_count += 1
        future = Future()
        future.set_result(NS(goals_canceling=[object()]))
        return future


class Logger:
    def error(self, message):
        pass

    def warn(self, message):
        pass


class ActionTest(unittest.TestCase):
    def setUp(self):
        self.client = Client()
        self.slot = ActionSlot(self.client, Logger())
        self.results = []

    def done(self, *args):
        self.results.append(args)

    def test_cancel_before_acceptance_cancels_late_goal_once(self):
        self.assertTrue(self.slot.send(object(), self.done))
        self.slot.cancel()
        self.assertFalse(self.slot.send(object(), self.done))
        handle = Handle()
        self.client.future.set_result(handle)
        self.slot.cancel()
        self.assertEqual(handle.cancel_count, 1)
        handle.result.set_result(NS(status=5))
        self.assertFalse(self.slot.busy)
        self.assertTrue(self.results[0][2])

    def test_rejection_is_reported_without_claiming_success(self):
        self.slot.send(object(), self.done)
        self.client.future.set_result(NS(accepted=False))
        self.assertFalse(self.slot.busy)
        self.assertIsNone(self.results[0][0])
        self.assertEqual(self.results[0][1], 'Goal rejected')

    def test_success_requires_terminal_result(self):
        self.slot.send(object(), self.done)
        handle = Handle()
        self.client.future.set_result(handle)
        self.assertTrue(self.slot.busy)
        self.assertFalse(self.slot.send(object(), self.done))
        handle.result.set_result(NS(status=4))
        self.assertFalse(self.slot.busy)
        self.assertEqual(self.results[0][0].status, 4)

    def test_unknown_result_locks_out_new_goals(self):
        self.slot.send(object(), self.done)
        handle = Handle()
        self.client.future.set_result(handle)
        handle.result.set_exception(RuntimeError('network lost'))
        self.assertTrue(self.slot.busy)
        self.assertFalse(self.results)


if __name__ == '__main__':
    unittest.main()
