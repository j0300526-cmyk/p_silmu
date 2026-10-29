"""Hand target bridge: direct IK, confirmed MoveIt plans, or MoveIt Servo JointJog."""
import json
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import JointJog
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from .action_slot import ActionSlot
from .differential_ik import DifferentialIK, DifferentialIKConfig
from .kinematics import load_arm_model
from .safety import InputGate, limited_step, smooth_duration, stamp_is_fresh


class RobotBridgeNode(Node):
    def __init__(self):
        super().__init__('robot_bridge_node')
        defaults = {
            'robot_description': '', 'mode': 'servo', 'input_timeout': 0.35,
            'joint_timeout': 0.5, 'control_rate': 20.0, 'target_frame': 'link1',
            'pitch_candidates': [0.0, 0.4, 0.8, -0.4, 1.2],
            'max_joint_velocity': 0.4, 'max_joint_acceleration': 0.8,
            'max_joint_step': 0.08, 'move_time': 0.3, 'servo_gain': 2.0,
            'gripper_open': 0.019, 'gripper_close': -0.010,
            'gripper_max_effort': 1.0, 'planning_time': 3.0,
            'velocity_scaling': 0.2, 'acceleration_scaling': 0.2,
            'servo_solver': 'cbf_qp', 'max_cartesian_speed': 0.06,
            'joint_limit_margin': 0.08, 'barrier_gain': 2.0,
            'damping_min': 0.01, 'damping_max': 0.06,
            'singularity_threshold': 0.04, 'temporal_weight': 0.0002,
            'posture_weight': 0.0,
            'gripper_joint_name': 'gripper_left_joint',
            'speed_profile': 'conservative',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = lambda name: self.get_parameter(name).value
        self.mode = self.p('mode')
        if self.mode not in ('direct_ik', 'planned', 'servo'):
            raise ValueError('mode must be direct_ik, planned, or servo')
        if self.p('servo_solver') not in ('cbf_qp', 'analytic'):
            raise ValueError('servo_solver must be cbf_qp or analytic')
        for name in ('control_rate', 'input_timeout', 'joint_timeout', 'max_joint_velocity',
                     'max_joint_acceleration', 'max_joint_step', 'move_time', 'servo_gain',
                     'planning_time', 'gripper_max_effort'):
            if not math.isfinite(self.p(name)) or self.p(name) <= 0:
                raise ValueError(name + ' must be finite and positive')
        for name in ('velocity_scaling', 'acceleration_scaling'):
            if not 0 < self.p(name) <= 1:
                raise ValueError(name + ' must be in (0, 1]')
        self.model = load_arm_model(self.p('robot_description'))
        self.joints = self.model.joint_names
        root = ET.fromstring(self.p('robot_description'))
        grip = next((j for j in root.findall('joint') if j.get('name') == self.p('gripper_joint_name')), None)
        if grip is None:
            raise ValueError('Gripper joint is absent from the selected robot URDF')
        grip_limit = grip.find('limit')
        lo, hi = float(grip_limit.get('lower')), float(grip_limit.get('upper'))
        if not lo <= self.p('gripper_close') <= self.p('gripper_open') <= hi:
            raise ValueError('Gripper targets must match the selected robot URDF limits and units')
        self.differential_ik = DifferentialIK(self.model, DifferentialIKConfig(
            gain=self.p('servo_gain'), max_cartesian_speed=self.p('max_cartesian_speed'),
            max_velocity=self.p('max_joint_velocity'),
            max_acceleration=self.p('max_joint_acceleration'),
            joint_margin=self.p('joint_limit_margin'), barrier_gain=self.p('barrier_gain'),
            damping_min=self.p('damping_min'), damping_max=self.p('damping_max'),
            singularity_threshold=self.p('singularity_threshold'),
            temporal_weight=self.p('temporal_weight'), posture_weight=self.p('posture_weight')))
        if self.mode == 'servo' and self.p('servo_solver') == 'cbf_qp' and self.p('control_rate') < 20:
            raise ValueError('cbf_qp requires control_rate >= 20 Hz (maximum period 0.1 s)')
        self.solver_diagnostics = {}
        self.input_age_ms = None
        self.gate = InputGate(self.p('input_timeout'), self.p('joint_timeout'))
        self.q = None
        self.target = None
        self.desired_gripper = None
        self.gripper_at = -math.inf
        self.completed_gripper = None
        self.servo_velocity = [0.0] * len(self.joints)
        self.servo_started = False
        self.start_pending = False
        self.start_retry_at = 0.0
        self.last_tick = time.monotonic()
        self.was_ready = False
        self.status_text = 'Waiting for tracking and joint feedback'
        self.status_pub = self.create_publisher(String, '~/status', 1)
        self.jog_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 1)
        arm_type = MoveGroup if self.mode == 'planned' else FollowJointTrajectory
        arm_name = '/move_action' if self.mode == 'planned' else '/arm_controller/follow_joint_trajectory'
        self.arm = ActionSlot(ActionClient(self, arm_type, arm_name), self.get_logger())
        self.gripper = ActionSlot(
            ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd'), self.get_logger())
        self.servo_start = self.create_client(Trigger, '/servo_node/start_servo')
        self.create_subscription(PoseStamped, '/hand_robot_pose', self.pose_cb, 1)
        self.create_subscription(Bool, '/hand_control/enabled', self.enable_cb, 1)
        self.create_subscription(String, '/hand_gripper', self.gripper_cb, 1)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, qos_profile_sensor_data)
        self.create_service(Trigger, '~/execute', self.execute_cb)
        self.create_service(Trigger, '~/stop', self.stop_cb)
        self.create_timer(1.0 / self.p('control_rate'), self.tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Bridge mode: ' + self.mode)
        self.get_logger().info('Speed profile: ' + self.p('speed_profile')
                               + f" | joint velocity {self.p('max_joint_velocity'):.3f} rad/s"
                               + f" | acceleration {self.p('max_joint_acceleration'):.3f} rad/s^2")

    def ros_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def joint_cb(self, msg):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if not stamp_is_fresh(stamp, self.ros_seconds(), self.p('joint_timeout')):
            return
        positions = dict(zip(msg.name, msg.position))
        if not all(n in positions for n in self.joints):
            return
        q = [positions[n] for n in self.joints]
        if not all(math.isfinite(v) and lo - 0.02 <= v <= hi + 0.02
                   for v, (lo, hi) in zip(q, self.model.limits)):
            return
        self.q = q
        self.gate.joints_at = time.monotonic()

    def enable_cb(self, msg):
        was_enabled = self.gate.enabled
        self.gate.set_enabled(msg.data, time.monotonic())
        if not msg.data:
            self.desired_gripper = None
            self.completed_gripper = None
            # A new enable must be accompanied by a new pose.
            self.gate.pose_at = -math.inf
            if was_enabled:
                self.halt()
        elif not was_enabled:
            self.status_text = 'Enabled; waiting for fresh target'

    def pose_cb(self, msg):
        xyz = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if (msg.header.frame_id != self.p('target_frame')
                or not all(math.isfinite(v) for v in xyz)
                or not stamp_is_fresh(stamp, self.ros_seconds(), self.p('input_timeout'))):
            self.gate.pose_at = -math.inf
            self.status_text = 'Rejected target frame, timestamp, or coordinates'
            return
        # Position-only contract; orientation is intentionally not an IK constraint.
        self.target = xyz
        self.input_age_ms = max(0.0, (self.ros_seconds() - stamp) * 1000)
        self.gate.pose_at = time.monotonic()

    def gripper_cb(self, msg):
        if msg.data not in ('GRASP', 'RELEASE'):
            self.get_logger().warn('Unknown gripper command ignored', throttle_duration_sec=2.0)
            return
        self.desired_gripper = msg.data
        self.gripper_at = time.monotonic()

    def target_joints(self):
        if self.q is None or self.target is None:
            return None
        return self.model.solve_position(self.target, self.q, self.p('pitch_candidates'))

    def send_jog(self, velocities):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p('target_frame')
        msg.joint_names = list(self.joints)
        msg.velocities = list(velocities)
        msg.duration = 0.15
        self.jog_pub.publish(msg)

    def halt(self):
        self.arm.cancel()
        self.gripper.cancel()
        self.servo_velocity = [0.0] * len(self.joints)
        if self.mode == 'servo':
            self.send_jog(self.servo_velocity)

    def stop_cb(self, request, response):
        self.gate.stop()
        self.halt()
        response.success = True
        response.message = 'Stop requested. Disable and re-enable hand control to rearm.'
        self.status_text = response.message
        return response

    def execute_cb(self, request, response):
        if self.mode != 'planned':
            response.message = 'execute is only available in planned mode'
        elif not self.gate.ready(time.monotonic()):
            response.message = 'Tracking, target, and joint feedback must be fresh and enabled'
        elif self.arm.busy:
            response.message = 'A plan/execution is already outstanding'
        else:
            q = self.target_joints()
            if q is None:
                response.message = 'No IK solution within loaded URDF limits'
            else:
                goal = MoveGroup.Goal()
                goal.request.group_name = 'arm'
                goal.request.pipeline_id = 'ompl'
                goal.request.planner_id = 'RRTConnectkConfigDefault'
                goal.request.num_planning_attempts = 5
                goal.request.allowed_planning_time = self.p('planning_time')
                goal.request.max_velocity_scaling_factor = self.p('velocity_scaling')
                goal.request.max_acceleration_scaling_factor = self.p('acceleration_scaling')
                goal.request.start_state.is_diff = True
                constraints = Constraints()
                for name, value in zip(self.joints, q):
                    jc = JointConstraint()
                    jc.joint_name, jc.position = name, float(value)
                    jc.tolerance_above = jc.tolerance_below = 0.01
                    jc.weight = 1.0
                    constraints.joint_constraints.append(jc)
                goal.request.goal_constraints = [constraints]
                goal.planning_options.plan_only = False
                goal.planning_options.replan = False
                goal.planning_options.planning_scene_diff.is_diff = True
                goal.planning_options.planning_scene_diff.robot_state.is_diff = True
                response.success = self.arm.send(goal, self.arm_done)
                response.message = ('Goal submitted; result is published on ~/status'
                                    if response.success else 'MoveIt action unavailable or retry pending')
        self.status_text = response.message
        return response

    def arm_done(self, response, error, cancelled):
        success = (response is not None and response.status == GoalStatus.STATUS_SUCCEEDED)
        if success:
            success = (response.result.error_code.val == 1 if self.mode == 'planned'
                       else response.result.error_code == 0)
        self.status_text = ('Arm goal cancelled' if cancelled else
                            'Arm goal succeeded' if success else 'Arm goal failed: ' + str(error or response))
        if not success and not cancelled:
            self.arm.retry_at = time.monotonic() + 1.0
            self.get_logger().warn(self.status_text)

    def gripper_done(self, desired, response, error, cancelled):
        if cancelled:
            return
        if (response is not None and response.status == GoalStatus.STATUS_SUCCEEDED
                and response.result.reached_goal):
            self.completed_gripper = desired
        else:
            self.gripper.retry_at = time.monotonic() + 1.0
            self.get_logger().warn('Gripper did not reach goal; retrying only with fresh input')

    def maybe_start_servo(self):
        if (self.servo_started or self.start_pending
                or time.monotonic() < self.start_retry_at or not self.servo_start.service_is_ready()):
            return
        self.start_pending = True
        self.servo_start.call_async(Trigger.Request()).add_done_callback(self.servo_start_done)

    def servo_start_done(self, future):
        self.start_pending = False
        self.start_retry_at = time.monotonic() + 1.0
        try:
            self.servo_started = future.result().success
        except Exception as exc:
            self.get_logger().warn('Servo start failed: ' + str(exc))

    def tick(self):
        now = time.monotonic()
        elapsed = now - self.last_tick
        dt = min(0.1, max(0.0, elapsed))
        self.last_tick = now
        if self.mode == 'servo':
            self.maybe_start_servo()
        ready = self.gate.ready(now)
        if not ready:
            if self.was_ready and self.gate.enabled:
                self.gate.stop()
                self.status_text = 'Input/feedback lost; disable and re-enable to rearm'
            self.halt()
        else:
            if self.mode == 'servo' and self.p('servo_solver') == 'cbf_qp':
                if self.servo_started:
                    solver_start = time.perf_counter()
                    result = self.differential_ik.step(self.q, self.target, self.servo_velocity, elapsed)
                    self.solver_diagnostics = {
                        'feasible': result.feasible, 'sigma_min': result.sigma_min,
                        'damping': result.damping, 'tracking_error_m': result.tracking_error,
                        'active_bounds': result.active_bounds,
                        'solve_ms': (time.perf_counter() - solver_start) * 1000,
                        'reason': result.reason,
                    }
                    if result.feasible:
                        self.servo_velocity = result.velocity
                        self.send_jog(self.servo_velocity)
                    else:
                        self.gate.stop()
                        ready = False
                        self.status_text = 'Differential IK stopped: ' + result.reason
                        self.halt()
            elif self.mode != 'planned':
                q = self.target_joints()
                if q is None:
                    self.status_text = 'No reachable IK target; motion stopped'
                    ready = False
                    self.halt()
                elif self.mode == 'direct_ik' and not self.arm.busy:
                    target = limited_step(self.q, q, self.p('max_joint_step'))
                    if max(abs(a - b) for a, b in zip(self.q, target)) > 0.002:
                        goal = FollowJointTrajectory.Goal()
                        goal.trajectory.joint_names = list(self.joints)
                        start = JointTrajectoryPoint()
                        start.positions = list(self.q)
                        start.velocities = start.accelerations = [0.0] * len(self.joints)
                        start.time_from_start = Duration(seconds=0.0).to_msg()
                        end = JointTrajectoryPoint()
                        end.positions = target
                        end.velocities = end.accelerations = [0.0] * len(self.joints)
                        duration = smooth_duration(self.q, target, self.p('move_time'),
                                                   self.p('max_joint_velocity'),
                                                   self.p('max_joint_acceleration'))
                        end.time_from_start = Duration(seconds=duration).to_msg()
                        goal.trajectory.points = [start, end]
                        self.arm.send(goal, self.arm_done)
                elif self.mode == 'servo' and self.servo_started:
                    cap = self.p('max_joint_velocity')
                    step = self.p('max_joint_acceleration') * dt
                    desired = [max(-cap, min(cap, self.p('servo_gain') * (t - a)))
                               for t, a in zip(q, self.q)]
                    self.servo_velocity = [a + max(-step, min(step, b - a))
                                           for a, b in zip(self.servo_velocity, desired)]
                    self.send_jog(self.servo_velocity)
            if (ready and self.desired_gripper is not None
                    and self.desired_gripper != self.completed_gripper
                    and now - self.gripper_at <= self.p('input_timeout')):
                desired = self.desired_gripper
                goal = GripperCommand.Goal()
                goal.command.position = self.p('gripper_close') if desired == 'GRASP' else self.p('gripper_open')
                goal.command.max_effort = self.p('gripper_max_effort')
                self.gripper.send(goal, lambda result, error, cancelled:
                                  self.gripper_done(desired, result, error, cancelled))
            elif not ready or now - self.gripper_at > self.p('input_timeout'):
                self.gripper.cancel()
        self.was_ready = ready
        self.status_pub.publish(String(data=json.dumps({
            'mode': self.mode, 'ready': ready, 'enabled': self.gate.enabled,
            'arm_busy': self.arm.busy, 'gripper_busy': self.gripper.busy,
            'servo_started': self.servo_started, 'message': self.status_text,
            'servo_solver': self.p('servo_solver'), 'input_age_ms': self.input_age_ms,
            'speed_profile': self.p('speed_profile'),
            'control_diagnostics': self.solver_diagnostics,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.gate.stop()
                node.halt()
                deadline = time.monotonic() + 0.7
                while (node.arm.busy or node.gripper.busy) and rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
