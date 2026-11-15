# Quadruped walk refactor and validation

## Architecture

```text
keyboard_input.py -> keyboard_walk.py -> gait_generator.py -> kinematics.py -> JointTrajectory
                                    |-> balance.py (COM/support margin)
                                    |-> safety_monitor.py <- /imu + foot contacts
                                    |-> robot_logger.py (independent CSV diagnostics)
```

`keyboard_walk.py` remains the `ros2 run open_quadruped keyboard_walk.py` entry point. It publishes the existing `/quadruped_controller/joint_trajectory` and preserves joint names. The pure modules have no ROS dependency except `kinematics.py`, which gets its geometry, inertias, and limits from the expanded URDF. The keyboard is rejected until all four contacts are present and the measured bent pose/IMU pitch are stable for two wall-clock seconds. `r` requests `/gazebo/set_model_state` and commands the validated bent joint pose; movement stays gated until stability returns. `x` remains disabled.

Safety thresholds are in `nodes/safety_monitor.py`: warning at 0.25 rad, fall at 0.65 rad, below two contacts is FALL, and unexpected contact loss while walking is WARNING. The planned swing foot is excluded from the contact check during its planned swing interval. WARNING stops the command and requests the measured stand pose. FALL zeros walking commands and reports a latched-looking movement refusal through stability gating; `r` is the recovery action.

## Unit and build results

- `colcon build --packages-select open_quadruped --symlink-install`: passed.
- `python3 -m pytest -q src/open_quadruped/test` (ROS Humble and workspace sourced): 5 passed.
- Coverage: URDF FK/IK round trip (<1 mm), joint limits, crawl swing count and periodicity, support margins, safety classifications, and trajectory seam continuity.

## Gazebo evidence

| Test | Measurement | Result |
|---|---|---|
| Static stand, stable window t=30–60 s | x range 0.052 mm, y 0.038 mm, z 0.034 mm; roll range 0.000288 rad; pitch range 0.000030 rad; all four contact sensors true for 100% of 1,501 samples | Pass for this 30 s stable interval. The initial placement transient before t=30 s is excluded. Logger: `logs/20261115_151054_trial.csv`. |
| IMU smoke test | `/imu` produced `base_link` orientation, angular velocity, and acceleration messages; observed acceleration z=9.7804 m/s² | Pass. |
| Keyboard `w` reproduction | At 51.3 s logger reported BL contact loss during setup; command was accepted only after the 2 s stable gate. Controller stopped before FL liftoff: support margin −5.4 mm vs required +22 mm. Logger also recorded BL upper-leg effort saturation at −4 Nm. Body remained nearly level (roll extrema −0.0196/+0.0163 rad); no full gait cycle completed. | Fail to walk; stop behaved as designed. Logger: `logs/20261115_151237_trial.csv`; keyboard report is in the launch output. |
| Crawl 10 cycles; calibration steps 5/10/20/30 mm; reverse/lateral/turn; stop/recovery perturbation; faster cycles; 1 m x3 | Not run: initial crawl cannot clear its measured support-margin gate, so these results would not be valid. Earlier saved calibration report records a world-anchor IK failure (FR target 2.0 mm beyond convergence threshold) and prior reverse drift. | Not validated. |

No fall cause can be assigned conclusively to the historical side fall from current logs. The reproduced attempt does show a concrete immediate blocker: before lifting FL, the COM margin was already −5.4 mm and BL upper-leg effort hit the 4 Nm limit. Earlier experiments also measured foot slip and found world-frame FR IK reach failures. These measured support/kinematic limits are consistent with the previous fall report, but do not prove its single root cause. No gain, effort limit, or STL was changed.

## Commands

```bash
cd ~/robodog_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch open_quadruped crawl_sim.launch.py gui:=false
```

In separate sourced terminals, run the logger and keyboard:

```bash
ros2 run open_quadruped robot_logger.py
ros2 run open_quadruped keyboard_walk.py
```

Unit/build checks:

```bash
source /opt/ros/humble/setup.bash
cd ~/robodog_ws
colcon build --packages-select open_quadruped --symlink-install
source install/setup.bash
python3 -m pytest -q src/open_quadruped/test
```

## Remaining limits

The quadruped did not walk in the measured `w` reproduction; no calibrated distance ratio or stable walking speed can be reported. Recovery via `r` is implemented but was not tested after an induced fall. Safety thresholds require hardware/model validation. The logger should be started before each Gazebo trial, and Gazebo should be stopped between trials.
