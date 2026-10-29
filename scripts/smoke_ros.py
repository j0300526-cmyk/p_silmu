#!/usr/bin/env python3
"""Real ROS integration smoke test. Requires a built workspace; no camera/hardware."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from omx_teleop.kinematics import load_arm_model
from omx_teleop.profiles import get_robot_profile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=['fake', 'gazebo_classic'], default='fake')
    parser.add_argument('--mode', choices=['planned', 'servo', 'direct_ik'], default='planned')
    parser.add_argument('--robot-model', choices=['omx_f', 'open_manipulator_x'], default='omx_f')
    parser.add_argument('--speed-check', action='store_true', help='Measure a 0.7-rad planned yaw move')
    parser.add_argument('--speed-report', type=Path, default=Path('artifacts/install/official_speed_motion.json'))
    args = parser.parse_args()
    if args.speed_check and (args.mode != 'planned' or args.robot_model != 'omx_f'):
        parser.error('--speed-check requires OMX-F planned mode')
    profile = get_robot_profile(args.robot_model)
    log = tempfile.NamedTemporaryFile(prefix='omx_smoke_', suffix='.log', delete=False)
    process = subprocess.Popen([
        'ros2', 'launch', 'omx_bringup', 'bringup.launch.py', 'backend:=' + args.backend,
        'mode:=' + args.mode, 'hand:=false', 'rviz:=false', 'gui:=false',
        'robot_model:=' + args.robot_model,
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('omx_smoke_probe', parameter_overrides=[
        Parameter('use_sim_time', value=args.backend == 'gazebo_classic')])
    state = {'q': None, 'status': None, 'model': None, 'gripper': None}
    subscriptions = []
    speed = {'collecting': False, 'desired_peak_velocity': 0.0,
             'desired_peak_acceleration': 0.0, 'actual_peak_velocity': 0.0,
             'motion_first_stamp': None, 'motion_last_stamp': None, 'samples': 0}
    speed_positions = []

    def controller_state(msg):
        if not speed['collecting']:
            return
        speed['samples'] += 1
        speed['desired_peak_velocity'] = max(speed['desired_peak_velocity'],
                                            max(map(abs, msg.desired.velocities), default=0.0))
        speed['desired_peak_acceleration'] = max(speed['desired_peak_acceleration'],
                                                max(map(abs, msg.desired.accelerations), default=0.0))
        speed['actual_peak_velocity'] = max(speed['actual_peak_velocity'],
                                           max(map(abs, msg.actual.velocities), default=0.0))
        if msg.actual.velocities and max(map(abs, msg.actual.velocities)) > .02:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            if speed['motion_first_stamp'] is None:
                speed['motion_first_stamp'] = stamp
            speed['motion_last_stamp'] = stamp

    def joints(msg):
        values = dict(zip(msg.name, msg.position))
        if all(n in values for n in profile.joints):
            state['q'] = [values[n] for n in profile.joints]
            if speed['collecting']:
                speed_positions.append((msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                                        values[profile.joints[0]]))
        state['gripper'] = values.get(profile.gripper_joint)

    def description(msg):
        state['model'] = load_arm_model(msg.data)

    subscriptions.append(node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(
        String, '/robot_description', description,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    subscriptions.append(node.create_subscription(
        String, '/robot_bridge_node/status', lambda msg: state.update(status=json.loads(msg.data)), 1))
    if args.speed_check:
        subscriptions.append(node.create_subscription(JointTrajectoryControllerState,
                             '/arm_controller/controller_state', controller_state, 10))
    enable = node.create_publisher(Bool, '/hand_control/enabled', 1)
    pose = node.create_publisher(PoseStamped, '/hand_robot_pose', 1)
    grip = node.create_publisher(String, '/hand_gripper', 1)
    stream = {'enabled': False, 'active': True, 'xyz': None, 'grip': 'RELEASE'}

    def pump():
        if process.poll() is not None:
            raise RuntimeError('Launch exited early')
        if stream['active']:
            enable.publish(Bool(data=stream['enabled']))
            if stream['enabled'] and stream['xyz'] is not None:
                msg = PoseStamped()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = profile.base_frame
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = stream['xyz']
                msg.pose.orientation.w = 1.0
                pose.publish(msg)
                grip.publish(String(data=stream['grip']))
        rclpy.spin_once(node, timeout_sec=0.03)

    def wait(predicate, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pump()
            if predicate():
                return
        raise TimeoutError('Smoke check timed out; state=' + str(state['status']))

    success = False
    try:
        wait(lambda: state['status'] is not None and state['model'] is not None
             and state['q'] is not None, 120)
        scene_client = node.create_client(GetPlanningScene, '/get_planning_scene')
        wait(scene_client.service_is_ready)
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        scene_future = scene_client.call_async(request)
        wait(scene_future.done)
        names = {obj.id for obj in scene_future.result().scene.world.collision_objects}
        assert {'workbench', 'obstacle'} <= names, names
        start = list(state['q'])
        target = list(start)
        target[0] += 0.7 if args.speed_check else 0.15
        stream.update(xyz=state['model'].forward(target)[:3], enabled=True)
        wait(lambda: state['status']['ready'])
        if args.mode == 'planned':
            client = node.create_client(Trigger, '/robot_bridge_node/execute')
            wait(client.service_is_ready)
            speed['collecting'] = args.speed_check
            future = client.call_async(Trigger.Request())
            wait(future.done)
            assert future.result().success, future.result().message
        wait(lambda: state['q'][0] - start[0] > 0.06, 30)
        if args.mode == 'planned':
            wait(lambda: not state['status']['arm_busy'], 30)
            assert 'succeeded' in state['status']['message'], state['status']
        if args.speed_check:
            speed['collecting'] = False
            # Gazebo's reported velocity may differ from its changing position.
            # Independently differentiate actual positions over >=80 ms windows.
            position_peak, first = 0.0, 0
            for last in range(len(speed_positions)):
                while first+1 < last and speed_positions[last][0]-speed_positions[first+1][0] >= .08:
                    first += 1
                dt = speed_positions[last][0]-speed_positions[first][0]
                if dt >= .08:
                    position_peak = max(position_peak, abs(
                        speed_positions[last][1]-speed_positions[first][1])/dt)
            moving = [(stamp, q) for stamp, q in speed_positions if abs(q-start[0]) > .002]
            reached = [(stamp, q) for stamp, q in moving if abs(q-start[0]-.7) < .015]
            speed.update(position_derived_peak_velocity=position_peak,
                         position_samples=len(speed_positions),
                         position_motion_duration_sim_sec=(reached[0][0]-moving[0][0]
                                                           if moving and reached else None))
            speed.update(robot_model=args.robot_model, backend=args.backend,
                         yaw_target_delta_rad=.7, yaw_actual_delta_rad=state['q'][0]-start[0],
                         nominal_velocity_limit_rad_s=.5, nominal_acceleration_limit_rad_s2=.5,
                         reference='official MoveIt limits 5.0 x default scale 0.1',
                         hardware_calibrated=False)
            assert speed['samples'] > 5, speed
            assert .1 < speed['desired_peak_velocity'] <= .5+.01, speed
            assert speed['desired_peak_acceleration'] <= .5+.03, speed
            assert speed['actual_peak_velocity'] <= .5+.15, speed
            assert .1 < position_peak <= .5+.15, speed
            assert abs(speed['yaw_actual_delta_rad']-.7) < .04, speed
            args.speed_report.parent.mkdir(parents=True, exist_ok=True)
            args.speed_report.write_text(json.dumps(speed, indent=2)+'\n')
            print('SPEED:', json.dumps(speed))
        stream['grip'] = 'GRASP'
        wait(lambda: state['gripper'] is not None and abs(state['gripper'] - profile.gripper_close) < .01, 15)
        # A dead camera stops publishing instead of sending a disable message.
        stream['active'] = False
        wait(lambda: not state['status']['ready'], 3)
        settle_at = time.monotonic() + 1.0
        wait(lambda: time.monotonic() >= settle_at, 3)
        held = list(state['q'])
        check_at = time.monotonic() + .6
        wait(lambda: time.monotonic() >= check_at, 2)
        assert max(abs(a-b) for a, b in zip(held, state['q'])) < .02, (held, state['q'])
        print('PASS: scene, arm motion, gripper, input-loss hold:', args.robot_model, args.backend, args.mode)
        success = True
    finally:
        if rclpy.ok():
            enable.publish(Bool(data=False))
            node.destroy_node()
            rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        log.close()
        if not success:
            print('Launch log:', log.name)
            print('\n'.join(Path(log.name).read_text(errors='replace').splitlines()[-100:]))


if __name__ == '__main__':
    main()
