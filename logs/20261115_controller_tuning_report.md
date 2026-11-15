# Position controller tuning — 2026-11-15

## Setup and mechanism

- Robot mass in expanded URDF: 3.14 kg. Leg links/paws kept at 0.10/0.03 kg.
- World step: 0.001 s. Solver settings were left at their baseline values for this sweep.
- Installed versions: `gazebo_ros2_control 0.4.10-1jammy.20261005.224036`, `controller_manager 2.54.3`, `ros2_controllers 2.54.1`.
- `quadruped_controller` sends `position` trajectories. Because `pid_gains.position` is configured in `ros2_controllers.yaml`, gazebo_ros2_control turns position error into PID effort and applies force to each Gazebo joint. This is not a direct position teleport. The configured effort limit is 4 Nm per joint.
- The target per leg was hip=0, upper=-0.68162889, lower=1.26606748 rad. URDF finite-difference Jacobian minimum singular value at this target: 0.0552 m/rad, just above the requested 0.05 threshold.

## Controlled sweep

| Variable | Result | Conclusion |
|---|---|---|
| 100 Hz, kp=2, kd=0.05 (previous baseline) | z≈0.137 m, pitch≈0.259 rad, max error≈0.43 rad; front lower leg reached its 1.5708 rad upper limit. | The target is not held at the original gains. |
| 500 Hz, kp=2, kd=0.05 | At 51.8 s sim: z=0.1369 m, pitch=0.2579 rad, max joint error=0.4287 rad; all four foot contacts remained on. | Raising the controller rate alone to 500 Hz did not fix the sag. |
| 1000 Hz, kp=5, kd=0.1 | Last 10 s: z=0.1860 m, pitch=0.0236 rad, max error=0.1559 rad, max effort=0.78 Nm; all contacts on. | Better posture and tracking, but error target not met. |
| 1000 Hz, kp=10, kd=0.1 | Last 10 s: z=0.1936 m, pitch=0.0076 rad, max error=0.0629 rad, max effort=0.63 Nm; all contacts on. | Close, but error target still missed. |
| 1000 Hz, kp=20, kd=0.1 | Stable window 40.0–79.4 s sim (39.4 s): mean z=0.19593 m, mean pitch=0.00473 rad, max error=0.03033 rad, max effort=0.607 Nm; all four contacts stayed on for the entire window. No effort saturation or joint limit. | Standing pose meets the requested ≈0.20 m, <0.05 rad, 30 s criteria. |
| Single foot, back-left, 10 mm target over 10 s | Three-foot COM margin=9.4 mm. The joint trajectory reached its target, but all four contact sensors continued reporting contact. Body height dropped about 4 mm while the foot moved in the body frame; estimated world-space foot-center rise was only about 5–6 mm. | The physical 10 mm lift is not verified. Do not start crawl yet. |

## Static torque and discrete damping checks

Rigid-body static estimate for the target pose (vertical foot reactions apportioned by the URDF COM, no horizontal force):

| Leg | Hip (Nm) | Upper (Nm) | Lower (Nm) |
|---|---:|---:|---:|
| Front left | -0.280 | -0.097 | -0.568 |
| Front right | +0.280 | -0.097 | -0.568 |
| Back left | -0.113 | -0.084 | -0.384 |
| Back right | +0.113 | -0.084 | -0.384 |

These are analytical estimates; actual contact-force distribution is not measured by this calculation. The strongest match is the loaded front knee: at the failed kp=2 point, `kp × error = 2 × (-0.305) ≈ -0.61 Nm`, close to the estimated -0.568 Nm static demand. In the validated kp=20 pose, the front lower joint's steady output is about -0.605 Nm, also close to that estimate. Raising kp from 2→5→10→20 reduced the maximum tracking error monotonically from about 0.43→0.156→0.063→0.030 rad.

Using URDF effective-inertia estimates (hip 0.00586, upper 0.00348, lower 0.000985 kg·m²), `kd·dt/Ieff` for the lower joint is ≈0.508 at 100 Hz with kd=0.05, ≈0.203 at 500 Hz with kd=0.1, and ≈0.102 at 1000 Hz with kd=0.1. Thus the tested 1000 Hz derivative setting is below the 0.5 guideline for all three joint groups. The earlier kd=0.2 at 100 Hz gives ≈2.03 for the lower joint and remains a credible explanation for the previous worsening with high kd; that particular unstable combination was not repeated.

## Saved trial files

- `20261115_040043_rate500_kp2_kd005.csv` and matching `_summary.txt`, `_params.txt`, `_plots/`
- `20261115_040156_rate1000_kp5_kd01.csv` and matching `_summary.txt`, `_params.txt`, `_plots/`
- `20261115_040333_rate1000_kp10_kd01.csv` and matching `_summary.txt`, `_params.txt`, `_plots/`
- `20261115_040458_rate1000_kp20_kd01.csv` and matching `_summary.txt`, `_params.txt`, `_plots/`

No GitHub push was made.
