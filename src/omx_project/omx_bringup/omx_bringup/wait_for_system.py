"""Startup gate: active controllers, fresh states, TF, actions, and advancing clock."""
import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


def main(args=None):
    rclpy.init(args=args)
    node = Node('wait_for_system')
    node.declare_parameter('timeout', 60.0)
    node.declare_parameter('require_clock', True)
    node.declare_parameter('required_joints',
                           ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint'])
    required_joints = node.get_parameter('required_joints').value
    timeout = node.get_parameter('timeout').value
    require_clock = node.get_parameter('require_clock').value
    state = {'joints': -math.inf, 'controllers': False, 'clock': None, 'advances': 0}
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    def joint_cb(msg):
        positions = dict(zip(msg.name, msg.position))
        if all(n in positions and math.isfinite(positions[n]) for n in required_joints):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            now = node.get_clock().now().nanoseconds / 1e9
            if -0.05 <= now - stamp <= 0.5:
                state['joints'] = time.monotonic()

    def clock_cb(msg):
        current = msg.clock.sec + msg.clock.nanosec / 1e9
        if state['clock'] is not None and current > state['clock']:
            state['advances'] += 1
        state['clock'] = current

    subscriptions = [
        node.create_subscription(JointState, '/joint_states', joint_cb, qos_profile_sensor_data),
        node.create_subscription(Clock, '/clock', clock_cb, qos_profile_sensor_data),
    ]
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    arm = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    gripper = ActionClient(node, GripperCommand, '/gripper_controller/gripper_cmd')
    future = None
    started = time.monotonic()
    success = False
    try:
        while rclpy.ok() and time.monotonic() - started < timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if future is None and client.service_is_ready():
                future = client.call_async(ListControllers.Request())
            if future is not None and future.done():
                response = future.result()
                active = {c.name for c in response.controller if c.state == 'active'}
                state['controllers'] = {'joint_state_broadcaster', 'arm_controller',
                                        'gripper_controller'} <= active
                future = None
            if (state['controllers'] and time.monotonic() - state['joints'] < 0.5
                    and (not require_clock or state['advances'] >= 2)
                    and buffer.can_transform('world', 'end_effector_link', Time())
                    and arm.server_is_ready() and gripper.server_is_ready()):
                success = True
                node.get_logger().info('Controllers, feedback, TF, and clock are ready')
                break
        if not success:
            node.get_logger().error('Startup timed out: ' + str(state))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1
