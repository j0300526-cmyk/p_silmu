"""MediaPipe hand input with explicit arming, freshness, and frame-rate independent motion."""
import json
import glob
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from omx_teleop.kinematics import load_arm_model
from omx_teleop.input_processing import (
    ControlHandSelector, DwellHysteresis, RelativeRetargeter, VectorFilter, hand_features,
)
from omx_teleop.safety import stamp_is_fresh
from omx_teleop.profiles import get_robot_profile


class HandControlNode(Node):
    def __init__(self):
        super().__init__('hand_control_node')
        for name, value in {
            'robot_description': '', 'camera_device': 'auto', 'show_window': True,
            'frame_rate': 30.0, 'max_speed': 0.06, 'deadzone_pixels': 40.0,
            'filter_window': 5, 'base_hand_size': 110.0,
            'pinch_close_ratio': 0.22, 'pinch_open_ratio': 0.30,
            'workspace_min': [0.05, -0.25, 0.05], 'workspace_max': [0.35, 0.25, 0.30],
            'mapping_mode': 'relative', 'input_filter': 'one_euro',
            'filter_min_cutoff': 1.0, 'filter_beta': 0.8, 'filter_derivative_cutoff': 1.0,
            'clutch_enabled': True, 'gesture_dwell': 0.08, 'precision_scale': 0.3,
            'relative_gains': [0.12, 0.4, 0.3], 'calibration_frames': 5,
            'calibration_tolerance': 0.05, 'max_target_lead': 0.04,
            'target_frame': 'auto',
            'hand_scheme': 'one_hand', 'control_hand': 'auto',
        }.items():
            self.declare_parameter(name, value)
        self.p = lambda name: self.get_parameter(name).value
        if self.p('hand_scheme') not in ('one_hand', 'two_hand'):
            raise ValueError('hand_scheme must be one_hand or two_hand')
        self.hand_selector = ControlHandSelector(self.p('control_hand'))
        self.selected_hand = None
        self.detected_hands = []
        if (self.p('frame_rate') <= 0 or self.p('max_speed') <= 0
                or self.p('filter_window') < 1 or not 0 < self.p('deadzone_pixels') < 100
                or not 0 < self.p('pinch_close_ratio') < self.p('pinch_open_ratio')):
            raise ValueError('Invalid hand control parameters')
        low, high = self.p('workspace_min'), self.p('workspace_max')
        if len(low) != 3 or len(high) != 3 or not all(a < b for a, b in zip(low, high)):
            raise ValueError('Invalid workspace bounds')
        if self.p('mapping_mode') not in ('relative', 'rate') or not 0 < self.p('precision_scale') <= 1:
            raise ValueError('Invalid mapping mode or precision scale')
        description = self.p('robot_description')
        if not description:
            # A separately started camera joins the active simulation's exact model.
            client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
            if not client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError('Start the simulation first: robot_state_publisher is unavailable')
            request = GetParameters.Request()
            request.names = ['robot_description']
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if not future.done() or future.result() is None or not future.result().values:
                raise RuntimeError('Could not obtain the active robot description')
            description = future.result().values[0].string_value
            self.destroy_client(client)
        self.model = load_arm_model(description)
        self.joints = self.model.joint_names
        profile = get_robot_profile('omx_f' if len(self.joints) == 5 else 'open_manipulator_x')
        self.window_title = profile.name + ' Hand Control'
        self.target_frame = profile.base_frame if self.p('target_frame') == 'auto' else self.p('target_frame')
        self.pose_pub = self.create_publisher(PoseStamped, '/hand_robot_pose', 1)
        self.gripper_pub = self.create_publisher(String, '/hand_gripper', 1)
        self.enable_pub = self.create_publisher(Bool, '/hand_control/enabled', 1)
        self.status_pub = self.create_publisher(String, '~/status', 1)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, qos_profile_sensor_data)
        self.create_service(SetBool, '~/set_enabled', self.enable_cb)
        self.execute_client = self.create_client(Trigger, '/robot_bridge_node/execute')
        self.armed = False
        self.tracking = False
        self.quit = False
        self.last_frame = time.monotonic()
        self.feedback_at = -math.inf
        self.current_q = None
        self.target = None
        self.gripper_state = 'RELEASE'
        self.precision = False
        self.status_text = 'DISARMED'
        self.start_button = None
        self.input_filter = VectorFilter(
            self.p('input_filter'), self.p('filter_window'),
            min_cutoff=self.p('filter_min_cutoff'), beta=self.p('filter_beta'),
            derivative_cutoff=self.p('filter_derivative_cutoff'))
        self.mapper = RelativeRetargeter(
            gains=self.p('relative_gains'), calibration_frames=self.p('calibration_frames'),
            calibration_tolerance=self.p('calibration_tolerance'), max_speed=self.p('max_speed'),
            max_lead=self.p('max_target_lead'), lower=low, upper=high)
        self.clutch = DwellHysteresis(self.p('pinch_close_ratio'), self.p('pinch_open_ratio'),
                                     self.p('gesture_dwell'))
        self.grasp = DwellHysteresis(self.p('pinch_close_ratio'), self.p('pinch_open_ratio'),
                                    self.p('gesture_dwell'))
        self.cap = None
        self.hands = None
        try:
            import cv2
            import mediapipe as mp
            self.cv2 = cv2
            if not hasattr(mp, 'solutions'):
                raise RuntimeError('Install the pinned MediaPipe version in requirements-vision.txt')
            self.hands = mp.solutions.hands.Hands(
                max_num_hands=2, min_detection_confidence=0.6, min_tracking_confidence=0.6)
            self.draw_landmarks = mp.solutions.drawing_utils.draw_landmarks
            self.hand_connections = mp.solutions.hands.HAND_CONNECTIONS
            devices = (sorted(glob.glob('/dev/video*'),
                              key=lambda path: int(path.removeprefix('/dev/video')))
                       if self.p('camera_device') == 'auto' else [self.p('camera_device')])
            for device in devices:
                cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
                if cap.isOpened() and cap.read()[0]:
                    self.cap = cap
                    self.get_logger().info('Camera connected: ' + device)
                    break
                cap.release()
            if self.cap is None:
                raise RuntimeError('No readable camera. Set camera_device or launch with hand:=false')
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if self.p('show_window'):
                cv2.namedWindow(self.window_title)
                cv2.setMouseCallback(self.window_title, self.mouse_cb)
        except Exception:
            self.close_resources()
            raise
        self.create_timer(1 / self.p('frame_rate'), self.process_frame,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Scheme: ' + self.p('hand_scheme')
                               + ' | SPACE: start/stop | PINCH: gripper | P: precision | ESC: quit')

    def joint_cb(self, msg):
        if len(msg.name) != len(msg.position):
            return
        positions = dict(zip(msg.name, msg.position))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        ros_now = self.get_clock().now().nanoseconds / 1e9
        if (all(n in positions and math.isfinite(positions[n]) for n in self.joints)
                and stamp_is_fresh(stamp, ros_now, 0.5)):
            self.current_q = [positions[n] for n in self.joints]
            self.feedback_at = time.monotonic()

    def enable_cb(self, request, response):
        self.set_armed(request.data)
        response.success = True
        response.message = 'Armed; visible control hand and fresh feedback required' if self.armed else 'Disarmed'
        return response

    def set_armed(self, enabled, status=None):
        self.armed = bool(enabled)
        self.hand_selector.reset()
        self.selected_hand = None
        self.clutch.reset()
        self.disable_tracking(status or ('ARMED / SHOW HAND AND HOLD STILL' if enabled else 'DISARMED'))

    def mouse_cb(self, event, x, y, flags, userdata):
        if event == self.cv2.EVENT_LBUTTONUP and self.start_button is not None:
            left, top, right, bottom = self.start_button
            if left <= x <= right and top <= y <= bottom:
                self.set_armed(not self.armed)

    def disable_tracking(self, status='HOLD / DISARMED'):
        self.tracking = False
        self.input_filter.reset()
        self.mapper.reset()
        self.grasp.reset()
        if status != self.status_text:
            self.get_logger().info(status)
        self.status_text = status
        self.enable_pub.publish(Bool(data=False))
        self.publish_status()

    def publish_status(self):
        age = time.monotonic() - self.feedback_at
        self.status_pub.publish(String(data=json.dumps({
            'armed': self.armed, 'tracking': self.tracking, 'clutch': self.clutch.state,
            'mapping_mode': self.p('mapping_mode'), 'filter': self.p('input_filter'),
            'precision': self.precision, 'calibrated': self.mapper.calibrated,
            'hand_scheme': self.p('hand_scheme'), 'control_hand': self.selected_hand,
            'detected_hands': self.detected_hands, 'gripper_command': self.gripper_state,
            'feedback_fresh': self.current_q is not None and age <= 0.5,
            'feedback_age_ms': round(age * 1000, 1) if math.isfinite(age) else None,
            'message': self.status_text,
        })))

    def normalized(self, value, deadzone, scale):
        if abs(value) <= deadzone:
            return 0.0
        return max(-1.0, min(1.0, (value - math.copysign(deadzone, value)) / max(scale, 1.0)))

    def process_frame(self):
        ok, frame = self.cap.read()
        capture_stamp = self.get_clock().now().to_msg()
        if not ok:
            self.set_armed(False, 'CAMERA LOST / PRESS SPACE TO REARM')
            self.get_logger().warn('Camera read failed; disarmed', throttle_duration_sec=2.0)
            return
        cv2 = self.cv2
        frame = cv2.flip(frame, 1)
        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2
        dz = self.p('deadzone_pixels')
        results = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self.last_frame))
        self.last_frame = now
        observations, ambiguous = {}, set()
        if results.multi_hand_landmarks and results.multi_handedness:
            for landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                points = [(lm.x * width, lm.y * height) for lm in landmarks.landmark]
                label = handedness.classification[0].label
                if label in observations:
                    ambiguous.add(label)
                else:
                    observations[label] = points
                if self.p('show_window'):
                    self.draw_landmarks(frame, landmarks, self.hand_connections)
        for label in ambiguous:
            observations.pop(label, None)
        self.detected_hands = sorted(observations)
        execute = False
        if self.p('show_window'):
            cv2.rectangle(frame, (int(center_x - dz), int(center_y - dz)),
                          (int(center_x + dz), int(center_y + dz)), (255, 255, 255), 1)
            scheme = 'ONE HAND' if self.p('hand_scheme') == 'one_hand' else 'TWO HANDS'
            state = 'TRACKING' if self.tracking else 'ARMED' if self.armed else 'STOPPED'
            cv2.putText(frame, scheme + ' / ' + state + (' / PRECISION' if self.precision else ''),
                        (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2)
            self.start_button = (width - 130, 8, width - 10, 42)
            cv2.rectangle(frame, self.start_button[:2], self.start_button[2:],
                          (30, 30, 180) if self.armed else (40, 130, 30), -1)
            cv2.putText(frame, 'STOP' if self.armed else 'START', (width - 112, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            # Keep the stop reason readable instead of clipping one long status line.
            for index, line in enumerate(self.status_text.split(' / ')):
                cv2.putText(frame, line, (15, 64 + 22 * index),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1)
            help_text = ('Click START or SPACE here | PINCH grip | P fine' if self.p('hand_scheme') == 'one_hand'
                         else 'Click START | LEFT PINCH clutch | RIGHT PINCH grip')
            cv2.putText(frame, help_text, (15, height - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, 'Control: ' + (self.selected_hand or 'show one hand')
                        + ' | Gripper: ' + self.gripper_state, (15, height - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow(self.window_title, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 32:
                self.set_armed(not self.armed)
            elif key in (10, 13):
                execute = True
            elif key in (ord('p'), ord('P')):
                self.precision = not self.precision
            elif key == 27:
                self.set_armed(False)
                self.quit = True
        one_hand = self.p('hand_scheme') == 'one_hand'
        if one_hand:
            self.selected_hand = self.hand_selector.update(observations, self.armed)
        else:
            self.selected_hand = 'Left' if self.armed and 'Left' in observations else None
        control_points = observations.get(self.selected_hand)
        gripper_points = control_points if one_hand else observations.get('Right')
        control_observation = None
        if control_points is not None:
            try:
                control_observation = hand_features(control_points, width, height)
            except ValueError:
                pass
        feedback_fresh = self.current_q is not None and now - self.feedback_at <= 0.5
        if not feedback_fresh:
            # Gazebo stops publishing joint states when paused (Space in its GUI).
            # Never resume motion automatically when feedback returns.
            self.set_armed(False, 'NO JOINT FEEDBACK / CHECK GAZEBO PLAY / THEN CLICK START HERE')
            return
        if self.status_text.startswith('NO JOINT FEEDBACK'):
            self.disable_tracking('FEEDBACK RESTORED / CLICK START OR PRESS SPACE HERE')
        valid = self.armed and control_observation is not None
        if not valid:
            if self.tracking or self.clutch.state or self.hand_selector.selected is not None:
                # Losing tracking requires deliberate rearming, rather than jumping on reacquisition.
                reason = ('HAND LOST' if control_points is None else 'HAND TOO SMALL OR INVALID')
                self.set_armed(False, reason + ' / SHOW WHOLE HAND AND CLICK START')
            else:
                self.clutch.reset()
                self.disable_tracking('ARMED / SHOW HAND' if self.armed else self.status_text)
            return
        features, pinch = control_observation
        engaged = one_hand or not self.p('clutch_enabled') or self.clutch.update(pinch, now)
        if not engaged:
            # Clutch release pauses and reindexes without disarming the keyboard enable.
            self.disable_tracking('ARMED / PINCH LEFT TO CLUTCH')
            return
        resolution = self.p('precision_scale') if self.precision else 1.0
        current_position = self.model.forward(self.current_q)[:3]
        previous_target = list(self.target) if self.target is not None else list(current_position)
        if self.p('mapping_mode') == 'relative':
            filtered = self.input_filter.update(features, now)
            target = self.mapper.update(filtered, current_position, now, resolution)
            if target is None:
                self.enable_pub.publish(Bool(data=False))
                self.status_text = 'CALIBRATING / HOLD HAND STILL'
                self.publish_status()
                return
            self.target = target
            self.tracking = True
        elif not self.tracking:
            self.target = list(self.model.forward(self.current_q)[:3])
            self.input_filter.reset()
            self.tracking = True
        else:
            wrist, middle = control_points[0], control_points[9]
            size = math.dist(wrist, middle)
            # Image x -> robot y, image y -> robot z, apparent size -> robot x.
            raw = [
                self.normalized(size - self.p('base_hand_size'), 15.0, 60.0),
                self.normalized(wrist[0] - center_x, dz, center_x - dz),
                self.normalized(center_y - wrist[1], dz, center_y - dz),
            ]
            low, high = self.p('workspace_min'), self.p('workspace_max')
            filtered = self.input_filter.update(raw, now)
            norm = math.sqrt(sum(v*v for v in filtered))
            for i, value in enumerate(filtered):
                step = value / max(1.0, norm) * self.p('max_speed') * resolution * dt
                # Permit an initial FK pose outside the input box without snapping to its boundary.
                proposed = self.target[i] + step
                if low[i] <= proposed <= high[i] or (
                        self.target[i] < low[i] and step > 0) or (
                        self.target[i] > high[i] and step < 0):
                    self.target[i] = proposed
        if self.model.solve_position(self.target, self.current_q, [0.0, 0.4, 0.8, -0.4, 1.2]) is None:
            self.target = previous_target
            if self.p('mapping_mode') == 'relative':
                self.mapper.target = list(previous_target)
        self.status_text = 'TRACKING'
        self.enable_pub.publish(Bool(data=True))
        pose = PoseStamped()
        pose.header.stamp = capture_stamp
        pose.header.frame_id = self.target_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.target
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        if gripper_points is not None:
            try:
                _, ratio = hand_features(gripper_points, width, height)
                self.gripper_state = 'GRASP' if self.grasp.update(ratio, now) else 'RELEASE'
                # Do not briefly issue RELEASE while the initial GRASP is still being debounced.
                if self.grasp.candidate is None:
                    self.gripper_pub.publish(String(data=self.gripper_state))
            except ValueError:
                self.grasp.reset()
        else:
            self.grasp.reset()
        self.publish_status()
        if execute:
            if self.execute_client.service_is_ready():
                self.execute_client.call_async(Trigger.Request()).add_done_callback(self.execute_done)
            else:
                self.get_logger().warn('Bridge execute service unavailable')

    def execute_done(self, future):
        try:
            result = future.result()
            self.get_logger().info(result.message)
        except Exception as exc:
            self.get_logger().warn('Execute request failed: ' + str(exc))

    def close_resources(self):
        if self.cap is not None:
            self.cap.release()
        if self.hands is not None:
            self.hands.close()
        if hasattr(self, 'cv2') and self.p('show_window'):
            self.cv2.destroyAllWindows()

    def destroy_node(self):
        if rclpy.ok():
            self.disable_tracking()
        self.close_resources()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HandControlNode()
        while rclpy.ok() and not node.quit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
