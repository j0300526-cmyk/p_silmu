# Synthetic ablation results

Synthetic signal and ideal Euler kinematics only; no physics/collision/user study.

Fixed seeds: 0, 1, 2, 3, 4.

| Filter | Stationary std (unitless) | Step 90% response (ms) |
| --- | ---: | ---: |
| moving_average | 0.01089 | 121.8 |
| one_euro | 0.00764 | 105.3 |

| Scenario | Method | RMS error (mm) | Margin violation steps / 300 | p95 solve (ms) |
| --- | --- | ---: | ---: | ---: |
| tracking | analytic | 9.74 | 0.0 | 0.069 |
| tracking | dls_clip | 9.72 | 0.0 | 0.042 |
| tracking | qp_no_temporal | 9.72 | 0.0 | 0.066 |
| tracking | cbf_qp | 9.72 | 0.0 | 0.066 |
| joint_margin | analytic | 2.14 | 295.0 | 0.040 |
| joint_margin | dls_clip | 2.15 | 295.0 | 0.039 |
| joint_margin | qp_no_temporal | 9.90 | 0.0 | 0.406 |
| joint_margin | cbf_qp | 9.90 | 0.0 | 0.418 |

The joint-margin scenario deliberately asks for a pose inside the physical joint range
but outside the configured buffer. Lower tracking error can therefore conflict with the buffer.
Runtime is local interpreter time, not a hard real-time or ROS end-to-end measurement.
