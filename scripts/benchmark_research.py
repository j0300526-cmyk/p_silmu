#!/usr/bin/env python3
"""Reproducible synthetic ablations; no ROS, dynamics, camera, or collision model."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import platform
import random
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/omx_project/omx_teleop'))
from omx_teleop.differential_ik import DifferentialIK, DifferentialIKConfig, solve_linear
from omx_teleop.input_processing import OneEuroFilter, VectorFilter
from omx_teleop.kinematics import ArmGeometry


def rms(values):
    return math.sqrt(sum(v*v for v in values) / len(values))


def percentile(values, fraction):
    values = sorted(values)
    return values[min(len(values) - 1, int((len(values) - 1) * fraction))]


def filter_trial(seed, method):
    rng = random.Random(seed)
    filt = OneEuroFilter() if method == 'one_euro' else VectorFilter('moving_average', 5)
    now, stationary, step_at, response_at = 0.0, [], None, None
    for _ in range(180):
        now += rng.uniform(.025, .041)
        signal = 0.0 if now < 2.0 else 1.0
        value = signal + rng.gauss(0, .025)
        output = (filt.update(value, now) if method == 'one_euro'
                  else filt.update([value]*3, now)[0])
        if .5 < now < 1.8:
            stationary.append(output)
        if signal == 1.0:
            if step_at is None:
                step_at = now
            if response_at is None and output >= .9:
                response_at = now
    return {'stationary_std': statistics.pstdev(stationary),
            'step_90_ms': 1000 * (response_at - step_at)}


def control_trial(model, seed, method, scenario):
    rng = random.Random(seed)
    cfg = DifferentialIKConfig(temporal_weight=0.0 if method == 'qp_no_temporal' else .0002)
    controller = DifferentialIK(model, cfg)
    q = [0.0, -1.0, .7, .3]
    if scenario == 'joint_margin':
        q[0] = model.limits[0][1] - .11
    previous = [0.0]*4
    errors, speeds, accelerations, durations = [], [], [], []
    violations = infeasible = 0
    minimum_margin = math.inf
    for step in range(300):
        dt = .05
        t = step * dt
        desired_q = ([.25 * math.sin(t*.8), -.9 + .08*math.sin(t*.6), .6, .3]
                     if scenario == 'tracking' else [model.limits[0][1] - .015, -1.0, .7, .3])
        target = [v + rng.gauss(0, .0005) for v in model.forward(desired_q)[:3]]
        started = time.perf_counter()
        if method in ('cbf_qp', 'qp_no_temporal'):
            result = controller.step(q, target, previous, dt)
            velocity = result.velocity
            infeasible += not result.feasible
        elif method == 'dls_clip':
            h, g, *_ = controller.objective(q, target, previous)
            unconstrained = solve_linear(h, [-v for v in g])
            velocity = [max(-cfg.max_velocity, min(cfg.max_velocity,
                        max(old - cfg.max_acceleration*dt, min(old + cfg.max_acceleration*dt, new))))
                        for old, new in zip(previous, unconstrained)]
        else:
            goal = model.solve_position(target, q, [0.0, .4, .8, -.4, 1.2])
            if goal is None:
                velocity = [0.0]*4
                infeasible += 1
            else:
                desired = [max(-cfg.max_velocity, min(cfg.max_velocity, cfg.gain*(b-a)))
                           for a, b in zip(q, goal)]
                velocity = [old + max(-cfg.max_acceleration*dt,
                                      min(cfg.max_acceleration*dt, new-old))
                            for old, new in zip(previous, desired)]
        durations.append((time.perf_counter() - started)*1000)
        q = [value + dt*speed for value, speed in zip(q, velocity)]
        margins = [min(value-lo, hi-value) for value, (lo, hi) in zip(q, model.limits)]
        minimum_margin = min(minimum_margin, *margins)
        violations += min(margins) < cfg.joint_margin - 1e-8
        errors.append(math.dist(model.forward(q)[:3], target))
        speeds.extend(abs(v) for v in velocity)
        accelerations.extend(abs(a-b)/dt for a, b in zip(velocity, previous))
        previous = velocity
    return {
        'tracking_rms_mm': 1000*rms(errors), 'margin_violation_steps': violations,
        'minimum_joint_margin_rad': minimum_margin, 'infeasible_steps': infeasible,
        'max_command_velocity_rad_s': max(speeds),
        'max_command_acceleration_rad_s2': max(accelerations),
        'solve_p95_ms': percentile(durations, .95), 'steps': len(errors),
    }


def aggregate(trials):
    keys = trials[0].keys()
    return {key: {'mean': statistics.mean(t[key] for t in trials),
                  'std': statistics.pstdev(t[key] for t in trials)} for key in keys}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seeds', type=int, default=5)
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/research_benchmark.json')
    args = parser.parse_args()
    if args.seeds < 1:
        parser.error('--seeds must be positive')
    model = ArmGeometry.from_urdf((ROOT / 'tests/fixtures/open_manipulator_x.urdf').read_text())
    methods = ('analytic', 'dls_clip', 'qp_no_temporal', 'cbf_qp')
    report = {
        'scope': 'Synthetic signal and ideal Euler kinematics only; no physics/collision/user study.',
        'python': platform.python_version(), 'seeds': list(range(args.seeds)),
        'controller_config': asdict(DifferentialIKConfig()),
        'filter_config': {'min_cutoff': 1.0, 'beta': .8, 'derivative_cutoff': 1.0,
                          'baseline_window': 5, 'noise_std': .025, 'unit_step_at_s': 2.0},
        'filter': {}, 'control': {},
    }
    for method in ('moving_average', 'one_euro'):
        report['filter'][method] = aggregate([filter_trial(seed, method) for seed in range(args.seeds)])
    for scenario in ('tracking', 'joint_margin'):
        report['control'][scenario] = {
            method: aggregate([control_trial(model, seed, method, scenario) for seed in range(args.seeds)])
            for method in methods}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    lines = ['# Synthetic ablation results', '', report['scope'], '',
             'Fixed seeds: ' + ', '.join(map(str, report['seeds'])) + '.', '',
             '| Filter | Stationary std (unitless) | Step 90% response (ms) |',
             '| --- | ---: | ---: |']
    for method, data in report['filter'].items():
        lines.append(f"| {method} | {data['stationary_std']['mean']:.5f} | {data['step_90_ms']['mean']:.1f} |")
    lines += ['', '| Scenario | Method | RMS error (mm) | Margin violation steps / 300 | p95 solve (ms) |',
              '| --- | --- | ---: | ---: | ---: |']
    for scenario, methods_data in report['control'].items():
        for method, data in methods_data.items():
            lines.append(f"| {scenario} | {method} | {data['tracking_rms_mm']['mean']:.2f} | "
                         f"{data['margin_violation_steps']['mean']:.1f} | {data['solve_p95_ms']['mean']:.3f} |")
    lines += ['', 'The joint-margin scenario deliberately asks for a pose inside the physical joint range',
              'but outside the configured buffer. Lower tracking error can therefore conflict with the buffer.',
              'Runtime is local interpreter time, not a hard real-time or ROS end-to-end measurement.']
    markdown = '\n'.join(lines) + '\n'
    args.output.with_suffix('.md').write_text(markdown)
    print(markdown)
    print('Full mean/std metrics:', args.output)


if __name__ == '__main__':
    main()
