"""Single owner of model, controllers, scene, and selected arm command path."""
import os
from pathlib import Path
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml

from omx_bringup.scene import load_objects, world_xml
from omx_teleop.profiles import get_robot_profile


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='omx_f',
                              choices=['omx_f', 'open_manipulator_x']),
        DeclareLaunchArgument('backend', default_value='gazebo_classic',
                              choices=['gazebo_classic', 'fake']),
        DeclareLaunchArgument('mode', default_value='servo',
                              choices=['servo', 'planned', 'direct_ik']),
        DeclareLaunchArgument('servo_solver', default_value='cbf_qp', choices=['cbf_qp', 'analytic']),
        DeclareLaunchArgument('mapping_mode', default_value='relative', choices=['relative', 'rate']),
        DeclareLaunchArgument('input_filter', default_value='one_euro',
                              choices=['one_euro', 'moving_average']),
        DeclareLaunchArgument('clutch', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('hand_scheme', default_value='one_hand', choices=['one_hand', 'two_hand']),
        DeclareLaunchArgument('control_hand', default_value='auto', choices=['auto', 'left', 'right']),
        DeclareLaunchArgument('speed_profile', default_value='auto',
                              choices=['auto', 'official', 'conservative']),
        DeclareLaunchArgument('hand', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('camera_device', default_value='auto'),
        DeclareLaunchArgument('show_camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('objects_file', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    backend, mode = value('backend'), value('mode')
    profile = get_robot_profile(value('robot_model'))
    official_speed = (value('speed_profile') == 'official' or
                      value('speed_profile') == 'auto' and profile.name == 'omx_f')
    if official_speed and profile.name != 'omx_f':
        raise ValueError('The official speed reference is for OMX-F only')
    sim = backend == 'gazebo_classic'
    shares = {name: Path(get_package_share_directory(name)) for name in (
        'omx_description', 'omx_control', 'omx_simulation', 'omx_moveit_config',
        'omx_hand_input', 'omx_teleop', 'open_manipulator_x_description')}
    read_yaml = lambda path: yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    control_file = shares['omx_control'] / 'config' / profile.control_directory / ('controllers_sim.yaml' if sim else 'controllers_fake.yaml')
    # Classic's SDF string conversion corrupts non-ASCII plugin parameter paths.
    # Keep the installed workspace anywhere, but pass an ASCII path to the plugin.
    tempdir = Path(tempfile.mkdtemp(prefix='omx_world_', dir='/tmp')) if sim else None
    if sim:
        runtime_control = tempdir / 'controllers.yaml'
        shutil.copyfile(control_file, runtime_control)
        control_file = runtime_control
    model = xacro.process_file(str(shares['omx_description'] / 'urdf' / profile.urdf_file),
                               mappings={'backend': backend, 'controllers_file': str(control_file)}).toxml()
    config = shares['omx_moveit_config'] / 'config'
    model_config = config / profile.control_directory
    limits_file = 'joint_limits.yaml'
    if profile.name == 'omx_f' and not official_speed:
        limits_file = 'joint_limits_conservative.yaml'
    bridge_files = [str(shares['omx_teleop'] / 'config/teleop.yaml')]
    hand_files = [str(shares['omx_hand_input'] / 'config/hand_control.yaml')]
    if official_speed:
        bridge_files.append(str(shares['omx_teleop'] / 'config/omx_f_official.yaml'))
        hand_files.append(str(shares['omx_hand_input'] / 'config/omx_f_official.yaml'))
    model_parameter = ParameterValue(model, value_type=str)
    common = {
        'use_sim_time': sim,
        'robot_description': model_parameter,
        'robot_description_semantic': ParameterValue(
            (config / profile.srdf_file).read_text(encoding='utf-8'), value_type=str),
        'robot_description_kinematics': read_yaml(config / 'kinematics.yaml'),
        'robot_description_planning': read_yaml(model_config / limits_file),
    }
    objects_file = value('objects_file') or str(shares['omx_simulation'] / 'config/workcell_objects.yaml')
    objects = load_objects(objects_file)
    move_params = {
        **common,
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': read_yaml(config / 'ompl_planning.yaml'),
        **read_yaml(model_config / 'moveit_controllers.yaml'),
        # Direct/Servo modes cannot be preempted by RViz Execute via move_group.
        'allow_trajectory_execution': mode == 'planned',
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 2.0,
        'trajectory_execution.allowed_start_tolerance': 0.03,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               name='robot_state_publisher', parameters=[common], output='screen')
    move_group = Node(package='moveit_ros_move_group', executable='move_group',
                      name='move_group', parameters=[move_params], output='screen')
    readiness = Node(package='omx_bringup', executable='wait_for_system',
                     parameters=[{'use_sim_time': sim, 'require_clock': sim, 'timeout': 60.0,
                                  'required_joints': [*profile.joints, profile.gripper_joint]}],
                     output='screen')
    scene = Node(package='omx_bringup', executable='scene_loader',
                 parameters=[{'use_sim_time': sim, 'objects_file': objects_file}], output='screen')
    bridge = Node(package='omx_teleop', executable='robot_bridge_node', name='robot_bridge_node',
                  parameters=[*bridge_files,
                              {'use_sim_time': sim, 'robot_description': model_parameter, 'mode': mode,
                               'servo_solver': value('servo_solver'), 'target_frame': profile.base_frame,
                               'gripper_joint_name': profile.gripper_joint,
                               'gripper_open': profile.gripper_open, 'gripper_close': profile.gripper_close}],
                  output='screen')
    final_nodes = [bridge]
    if mode == 'servo':
        servo_config = read_yaml(config / 'servo.yaml')
        servo_config['use_gazebo'] = sim
        servo = Node(package='moveit_servo', executable='servo_node_main', name='servo_node',
                     parameters=[common, {'moveit_servo': servo_config}], output='screen')
        final_nodes.insert(0, servo)
    if value('hand') == 'true':
        final_nodes.append(Node(
            package='omx_hand_input', executable='hand_control_node', name='hand_control_node',
            parameters=[*hand_files,
                        {'use_sim_time': sim, 'robot_description': model_parameter,
                         'camera_device': value('camera_device'),
                         'hand_scheme': value('hand_scheme'), 'control_hand': value('control_hand'),
                         'mapping_mode': value('mapping_mode'), 'input_filter': value('input_filter'),
                         'target_frame': profile.base_frame,
                         'clutch_enabled': value('clutch') == 'true',
                         'show_window': value('show_camera') == 'true'}], output='screen'))
    if value('rviz') == 'true':
        final_nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                                arguments=['-d', str(shares['omx_moveit_config'] / 'rviz/moveit.rviz')],
                                parameters=[common], output='screen'))

    def after_success(process, actions):
        def callback(event, ctx):
            if event.returncode != 0:
                return [LogInfo(msg='ERROR: Startup process failed; shutting down'),
                        EmitEvent(event=Shutdown(reason='Startup failure'))]
            return actions
        return RegisterEventHandler(OnProcessExit(target_action=process, on_exit=callback))

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '60', '--switch-timeout', '30'],
                    output='screen')

    broadcaster, arm, gripper = [spawner(n) for n in (
        'joint_state_broadcaster', 'arm_controller', 'gripper_controller')]
    actions = [
        after_success(broadcaster, [arm]),
        after_success(arm, [gripper]),
        after_success(gripper, [readiness]),
        after_success(readiness, [move_group, scene]),
        after_success(scene, final_nodes),
    ]
    persistent = [rsp, move_group, *final_nodes]
    if sim:
        world_file = tempdir / 'workcell.world'
        world_file.write_text(world_xml(objects), encoding='utf-8')
        actions.append(RegisterEventHandler(OnShutdown(
            on_shutdown=lambda event, ctx: shutil.rmtree(tempdir, ignore_errors=True))))
        model_path = os.pathsep.join(dict.fromkeys([
            str(shares['omx_description'].parent), str(shares['open_manipulator_x_description'].parent)]))
        if os.environ.get('GAZEBO_MODEL_PATH'):
            model_path += os.pathsep + os.environ['GAZEBO_MODEL_PATH']
        actions.append(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory('gazebo_ros')) / 'launch/gazebo.launch.py')),
            launch_arguments={'world': str(world_file), 'gui': value('gui'),
                              'pause': 'false', 'verbose': 'false',
                              'server_required': 'true'}.items())
        spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                     arguments=['-entity', 'omx', '-topic', 'robot_description', '-timeout', '60'],
                     output='screen')
        actions.extend([after_success(spawn, [broadcaster]), rsp, gazebo, spawn])
    else:
        control = Node(package='controller_manager', executable='ros2_control_node',
                       parameters=[{'robot_description': model_parameter, 'use_sim_time': False},
                                   str(control_file)], output='screen')
        persistent.append(control)
        actions.extend([rsp, control, broadcaster])
    for process in persistent:
        actions.insert(0, RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='A required node exited'))])))
    return actions
