# Franka Tracker Teleop Memory

Date: 2026-07-15

This note is the handoff memory for the Franka FR3 tracker teleoperation work.
It records the current control chain, debug tooling, launch commands, safety
status, and the next useful debugging steps.

Commands in this note use a relocatable project root and the `franka` conda
environment:

```bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate franka
export FRANKA_ROOT="${FRANKA_ROOT:-$HOME/franka_description}"
```

## Current Workspace

- Main ROS 2 workspace: `$FRANKA_ROOT/franka_ros2`
- ROS/conda environment: `$CONDA_PREFIX`
- Tracker/libsurvive source: `$FRANKA_ROOT/libsurvive`
- Tracker bridge package: `franka_tracker_bridge`
- Pose controller package: `franka_arm_controllers`

Use the conda activation path for interactive terminals:

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate franka
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
source install/setup.bash
```

## Current Runtime Chain

```text
libsurvive tracker
-> libsurvive_pose_node
-> /tracker/pose
/joint_states -> tracker_pose_target_node startup FK baseline
-> tracker_pose_target_node
-> /franka_controller/target_cartesian_pose
-> joint_impedance_pose_controller
-> official KDL IK
-> /franka_controller/kdl_desired_joint_states
-> tracker_meshcat_preview_node
-> Meshcat Franka preview
```

Optional debug dashboard:

```text
/tracker/pose
/tracker/deadman
/franka_controller/target_cartesian_pose
/franka_controller/kdl_desired_joint_states
-> tracker_debug_curve_node
-> Matplotlib debug curves
```

## Main Files

- `franka_tracker_bridge/franka_tracker_bridge/tracker_pose_target_core.py`
  - Relative tracker pose mapping, filtering, deadbanding, jump suppression.
- `franka_tracker_bridge/franka_tracker_bridge/tracker_pose_target_node.py`
  - Latches the first complete current arm JointState, computes the startup TCP with FK,
    then publishes `/franka_controller/target_cartesian_pose`.
- `franka_tracker_bridge/franka_tracker_bridge/debug_curve_core.py`
  - Debug curve buffers, quaternion-to-RPY conversion, target delta calculation.
- `franka_tracker_bridge/franka_tracker_bridge/tracker_debug_curve_node.py`
  - ROS/Matplotlib debug dashboard node.
- `franka_tracker_bridge/launch/tracker_preview.launch.py`
  - Starts tracker, target node, Meshcat preview, and optional debug curves.
- `franka_tracker_bridge/config/tracker_bridge_preview.yaml`
  - Main tracker mapping and preview defaults.
- `franka_arm_controllers/config/controllers.yaml`
  - Pose controller and KDL desired joint-state topic configuration.

## Relative Pose Algorithm

The tracker is not sent as an absolute robot pose. The target node first latches
the current seven arm joints from `/joint_states` and computes the robot start TCP
pose with FK. Startup calibration then records the tracker zero pose. Runtime
motion is relative:

```text
dp = p_tracker_now - p_tracker_start
dq = q_tracker_now * inverse(q_tracker_start)
```

The relative delta is mapped into a Franka target TCP pose, then published as an
absolute `geometry_msgs/msg/PoseStamped`.

Important implementation details:

- Tracker pose is low-pass filtered before target generation.
- Startup calibration averages the tracker start pose over the configured
  startup window.
- Deadman controls whether target poses are published; it does not reset the
  tracker start pose.
- Jump suppression limits per-frame target changes, not the total workspace.
- Franka inverse kinematics is still the controller-side KDL path. The tracker
  bridge does not solve Franka IK.

## Current Mapping Defaults

Important defaults in `tracker_bridge_preview.yaml`:

```yaml
coord_swap: [1, 0, 2]
coord_flip: [1.0, -1.0, 1.0]
coord_scale: [1.0, 1.0, 1.0]
base_xy_rotation_deg: 90.0
orientation_alignment_rpy_deg: [0.0, 0.0, 180.0]
tracker_rotation_scale: -1.0
tracker_rotation_axis_scale: [1.0, 1.0, -1.0]
tracker_rotation_axis_order: [0, 1, 2]
tracker_pos_soft_limit_mm: 15.0
tracker_pos_hard_limit_mm: 40.0
tracker_rot_soft_limit_deg: 6.0
tracker_rot_hard_limit_deg: 15.0
```

Meaning:

- Position starts as `robot_x <- tracker_y`, `robot_y <- -tracker_x`,
  `robot_z <- tracker_z`.
- XY is rotated by +90 degrees in the preview configuration to align tracker and Franka horizontal directions.
- Orientation is aligned with 180 degrees around z, then sign-corrected.

## Current Initial Franka Pose

There is no tracker-specific fixed Franka start configuration. At node startup:

1. `tracker_pose_target_node` waits for one complete, finite, in-limit seven-joint
   sample from `/joint_states`.
2. It computes the configured IK frame pose with Pinocchio FK and latches that pose once.
3. Tracker samples received before the joint baseline are discarded. Later JointState
   messages do not move the baseline.
4. Fake control starts from the mock hardware's reported current state, and Meshcat
   follows the controller's desired JointState output.

A missing or invalid startup JointState therefore fails closed instead of falling
back to a neutral model pose or configured q.

## Debug Curve Dashboard

Implemented node: `tracker_debug_curve_node`

Launch switch:

```bash
debug_curves:=true
```

Subscribed topics:

- `/tracker/pose`
- `/tracker/deadman`
- `/franka_controller/target_cartesian_pose`
- `/franka_controller/kdl_desired_joint_states`

Current plots:

- tracker `x/y/z`
- tracker `roll/pitch/yaw`
- target `x/y/z`
- target `roll/pitch/yaw`
- target delta `x/y/z`, measured from the first target pose sample
- target delta `roll/pitch/yaw`
- KDL `q_des[0..6]`
- deadman

Known display limitations:

- RPY is not a continuous orientation representation. Near `+pi/-pi`, roll can
  jump between about `+3.14` and `-3.14` even when the quaternion is continuous.
- Therefore, judge real attitude disturbance with target delta, KDL q_des, and
  future quaternion or axis-angle plots, not raw target RPY alone.
- The time axis currently uses epoch seconds, so Matplotlib may show a large
  `+1.78e9` offset. A relative-time axis would be easier to read.

## Current Diagnostic Conclusions

Recent debug plots showed:

- `tracker_low_pass_alpha:=0.08`, `translation_deadband:=0.01`, and
  `rotation_deadband:=0.05` improved static tracker input stability.
- `/franka_controller/target_cartesian_pose` had exactly one publisher:
  `tracker_pose_target_node`. The earlier target jumps were not caused by
  duplicate target publishers.
- `/franka_controller/kdl_desired_joint_states` is published by
  `joint_impedance_pose_controller`. After rebuilding/restarting the tracker
  preview, the debug curve dashboard displayed KDL q_des correctly.
- In unstable windows, tracker pose, target delta, and KDL q_des moved together.
  That points to tracker input or orientation mapping disturbance, not an
  isolated Meshcat plotting issue.
- Raw `target_rpy` can look much worse than the real quaternion because of
  Euler-angle wrapping. However, when `target_delta_rpy` and `q_des` also spike,
  the disturbance is real and unsafe for the robot.

## Real Robot Safety Status

Do not directly run this on the real Franka based on the latest plots.

Reason:

- Some windows still show target attitude spikes and multi-joint KDL q_des
  disturbance.
- The dashboard does not yet show `dq_des = diff(q_des) / dt`, which is the most
  direct signal for dangerous joint target velocity spikes.
- Deadman false/freeze behavior still needs a dedicated verification run.

Minimum gate before real hardware:

- Static tracker for 30-60 seconds: target xyz should stay essentially flat.
- Static tracker: target delta attitude should not show large spikes.
- Static tracker: KDL q_des should stay essentially flat.
- Add and inspect `dq_des[0..6]`; it must not show velocity spikes.
- Verify deadman false freezes or stops target updates as intended.
- Confirm exactly one publisher on `/franka_controller/target_cartesian_pose`.
- First real run must use low stiffness, low speed, small workspace motion, and
  an operator ready on the emergency stop.

## Recommended Launch Commands

Terminal 1, fake Franka plus pose controller:

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate franka
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
source install/setup.bash

ros2 launch franka_arm_controllers joint_impedance_pose_controller.launch.py robot_config_file:=tracker_fake_fr3_config.yaml controller_name:=joint_impedance_pose_controller
```

Terminal 2, tracker plus Meshcat plus debug curves:

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate franka
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
source install/setup.bash

ros2 launch franka_tracker_bridge tracker_preview.launch.py libsurvive_args:=--force-calibrate ik_frame_name:=fr3_link8 open_browser:=true debug_curves:=true tracker_low_pass_alpha:=0.08 translation_deadband:=0.01 rotation_deadband:=0.05
```

More conservative input filtering for another diagnostic run:

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py libsurvive_args:=--force-calibrate ik_frame_name:=fr3_link8 open_browser:=true debug_curves:=true tracker_low_pass_alpha:=0.04 translation_deadband:=0.02 rotation_deadband:=0.12
```

No tracker hardware, only verify ROS/Meshcat/debug chain:

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py use_libsurvive:=false open_browser:=true debug_curves:=true
```

## Useful Diagnostics

Check topic publishers/subscribers:

```bash
ros2 topic info -v /franka_controller/target_cartesian_pose
ros2 topic info -v /tracker/pose
ros2 topic info -v /franka_controller/kdl_desired_joint_states
ros2 node list
```

Expected:

- `/franka_controller/target_cartesian_pose` has one publisher:
  `tracker_pose_target_node`.
- `/franka_controller/kdl_desired_joint_states` has one publisher:
  `joint_impedance_pose_controller`.
- `tracker_debug_curve_node` subscribes to KDL q_des when `debug_curves:=true`.
- `tracker_meshcat_preview_node` uses
  `/franka_controller/kdl_desired_joint_states` as `joint_states_topic`.

Check node parameters:

```bash
ros2 node info /tracker_debug_curve_node
ros2 node info /tracker_meshcat_preview_node
ros2 param get /tracker_debug_curve_node kdl_desired_joint_states_topic
ros2 param get /tracker_meshcat_preview_node joint_states_topic
```

Check data and rates. Use separate commands or `timeout`; `ros2 topic hz` keeps
running until interrupted:

```bash
timeout 5s ros2 topic echo /franka_controller/kdl_desired_joint_states --once
timeout 5s ros2 topic hz /franka_controller/target_cartesian_pose
timeout 5s ros2 topic hz /franka_controller/kdl_desired_joint_states
```

## Verification Snapshot

Commands that passed in the current checkout from
`$FRANKA_ROOT/franka_ros2`:

```bash
timeout 180s conda run -n franka python -m py_compile   franka_tracker_bridge/franka_tracker_bridge/debug_curve_core.py   franka_tracker_bridge/franka_tracker_bridge/tracker_debug_curve_node.py   franka_tracker_bridge/scripts/tracker_debug_curve_node   franka_tracker_bridge/launch/tracker_preview.launch.py

timeout 180s conda run -n franka python   franka_tracker_bridge/test/test_debug_curve_core.py -v

timeout 180s conda run -n franka python   franka_tracker_bridge/test/test_debug_curve_node_static.py -v

timeout 180s conda run -n franka colcon build   --packages-select franka_tracker_bridge   --cmake-args -DBUILD_TESTING=ON

timeout 180s conda run -n franka colcon test   --packages-select franka_tracker_bridge   --event-handlers console_direct+

timeout 180s conda run -n franka colcon test-result --verbose
```

Observed result:

```text
franka_tracker_bridge: 13/13 package tests passed
Overall test-result summary: 91 tests, 0 errors, 0 failures, 3 skipped
```

The build emitted only a conda `gtest_vendor` CMake deprecation warning.

## Next Priorities

1. Add `dq_des[0..6] = diff(q_des) / dt` curves and max absolute joint target
   velocity. This is the most useful real-robot safety signal.
2. Add target quaternion `x/y/z/w` or target delta axis-angle norm to separate
   RPY display wrapping from real attitude jumps.
3. Change dashboard time axes to relative seconds from the first sample.
4. Add a deadman verification plot or state marker showing target freeze/stale
   behavior when deadman becomes false.
5. For real hardware, add actual-vs-desired joint curves, q_error, and Franka
   torque/state curves before enabling normal teleoperation.

## Historical Note

Earlier simulation tuning improved the Fairino-style mapping. Later debug curves
showed the chain is usable for diagnosis but still not ready for direct
real-robot operation without the safety gates above.
