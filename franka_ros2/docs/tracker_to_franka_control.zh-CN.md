# Tracker 到 Franka 控制链路 Meshcat 预览

本文档描述当前 tracker 控制验证链路。目标 pose 由 tracker 相对位姿生成并以 base 坐标系下的绝对位姿发布，pose 控制器用官方 KDL IK 解出期望关节角，并发布给 Meshcat 显示。Meshcat 不再默认自己做 Pinocchio IK。

执行本文命令前先激活环境并设置项目根目录：

```bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate franka
export FRANKA_ROOT="${FRANKA_ROOT:-$HOME/franka_description}"
```

## 总体链路

```text
tracker pose / libsurvive
-> libsurvive_pose_node
-> /tracker/pose
-> tracker_pose_target_node
-> /franka_controller/target_cartesian_pose
-> franka_arm_controllers/JointImpedancePoseController
-> KDL IK
-> /franka_controller/kdl_desired_joint_states
-> tracker_meshcat_preview_node
-> Meshcat: Franka FR3 机械臂 + tracker 虚拟末端坐标轴
```

这条链路按 tracker 的空间运动来做，不再把 tracker 当摇杆速度源。tracker 在空间中的相对变化量会作用到机械臂启动末端 pose 上，得到一个新的目标 pose。

## tracker_pose_target_node

`tracker_pose_target_node` 是当前主链路的算法节点。它订阅 tracker pose 和 deadman，输出 Franka 末端目标 pose：

```text
/tracker/pose
/tracker/deadman
/joint_states  # 启动时当前 7 关节角
-> tracker_pose_target_node
-> /franka_controller/target_cartesian_pose
```

映射逻辑：

```text
节点启动时锁存 /joint_states 中首个完整、有限且不越限的 7 关节状态，并通过 FK 计算 fr3_link8 的 robot_start_pose
节点启动后先对 tracker pose 做低通滤波
默认前 calibration_duration_sec=3.0 秒自动标定，记录 tracker 平均位姿作为 tracker_start
deadman 只控制是否输出目标 pose；不改变 tracker_start

p_start, q_start
p_curr,  q_curr

T_tracker_raw -> low-pass -> T_tracker_filtered
T_tracker_start = (R_ts, p_ts)
T_tracker_curr  = (R_tc, p_tc)  # current filtered tracker pose
T_robot_start   = (R_rs, p_rs)

T_tracker_rel = inverse(T_tracker_start) * T_tracker_curr

tracker_dp_local = R_ts^T * (p_tc - p_ts)
tracker_dR       = R_ts^T * R_tc
tracker_dR -> axis-angle rotation_vector

tracker_dp_local = deadband(tracker_dp_local, translation_deadband)
rotation_vector  = deadband(rotation_vector, rotation_deadband)

robot_dp_local = R_map * tracker_dp_local * translation_scale
if translation_limit[i] >= 0:
  robot_dp_local[i] = clamp(robot_dp_local[i], -translation_limit[i], translation_limit[i])

robot_rotation_vector = R_map * rotation_vector * rotation_scale
if rotation_limit < pi:
  robot_rotation_vector = clamp_norm(robot_rotation_vector, rotation_limit)

candidate_pose.position = p_rs + R_rs * robot_dp_local
candidate_pose.orientation = R_rs * exp(robot_rotation_vector)

robot target = candidate_pose
publish candidate_pose
```

关键点：

- 节点先等待 `/joint_states` 的完整实时 7 关节状态并锁存启动末端位姿；在此之前到达的 tracker pose 会被丢弃，不参与标定，也不会发布目标。
- 随后默认前 `calibration_duration_sec=3.0` 秒只用于滤波和自动标定；保持 tracker 和机械臂不动，节点会用这段时间的 tracker 平均位姿作为零点。如果设为 `0.0`，才退回 `calibration_sample_count` 帧标定模式。
- 当前对齐 Fairino/DexCap 风格：位置使用 tracker world 相对位移 `p_current - p_start`，姿态使用 left/world 增量 `R_current * R_start^-1`，再映射到 `T_robot_start` 上生成目标 pose。
- `tracker_low_pass_alpha` 对 tracker 原始位置和姿态做低通；越小越稳但延迟越大。
- `translation_deadband` 和 `rotation_deadband` 把小幅相对抖动置 0。
- 当前节点不做“无解点/工作空间超界”预过滤；它会持续发布相对启动基准生成的目标 pose，后续 KDL 控制层再处理能否跟随。
- 不直接使用 tracker 的绝对位姿；位置相对量采用世界坐标分量差，姿态相对量采用左乘旋转增量。
- 松开 deadman 后再次按下，不会重新建立 tracker 基准；重新打开后会按当前 tracker 相对启动基准生成目标 pose。需要重新标定时，重启节点，或后续添加显式 recalibration service/button。
- `translation_scale` 和 `rotation_scale` 控制 tracker 到机械臂目标的比例。
- 当前默认不做人为平移限幅；`translation_limit` 每轴为负数表示该轴不限幅。
- 当前默认不额外限制姿态相对量；`rotation_limit=pi` 等价于保留四元数最短相对旋转。
- `map_matrix` 控制坐标轴对应关系和正负号。
- `robot_start_pose` 由启动时实际 7 关节角的 FK 得到并只锁存一次；没有固定 q 或固定笛卡尔位姿，后续关节状态不会让基准漂移。

## tracker_meshcat_preview_node

`tracker_meshcat_preview_node` 订阅 `/franka_controller/target_cartesian_pose` 和 `/franka_controller/kdl_desired_joint_states`：

- 在 Meshcat 中显示 Franka FR3 机械臂模型。
- 显示 tracker 虚拟末端坐标轴，X 红、Y 绿、Z 蓝。
- 默认关闭自身 IK，只用 `/franka_controller/kdl_desired_joint_states` 驱动 FR3 模型。

这个节点自己用 `xacro` 展开 `franka_description` 里的 FR3 模型，再用 Meshcat 显示。它不依赖 RViz。

如果只启动 `tracker_preview.launch.py` 而不启动 pose 控制器，tracker 虚拟末端坐标轴仍会随目标 pose 移动，但 Franka 模型没有 KDL JointState 输入，不会跟随运动。

## Pose 型真机控制器

已在官方 `franka_arm_controllers` 中新增一个 pose 输入版控制器：

```text
tracker pose / libsurvive
-> libsurvive_pose_node
-> /tracker/pose
-> tracker_pose_target_node
-> /franka_controller/target_cartesian_pose
-> franka_arm_controllers/JointImpedancePoseController
-> KDL IK
-> /franka_controller/kdl_desired_joint_states
-> 关节阻抗力矩
-> Franka FR3
```

新增控制器保留官方关节阻抗控制思路中的核心部分：

- Franka state/model semantic interfaces。
- KDL `ChainIkSolverPos_NR_JL` 逆解。
- 每次 update 用当前 7 个关节位置作为 KDL 初值。
- 发布 `/franka_controller/kdl_desired_joint_states`，供 Meshcat 显示 KDL IK 解出的 FR3 关节姿态。
- 关节阻抗力矩：`Kp * (q_des - q) - Kd * dq_filtered + coriolis`。
- 默认 collision behavior service 调用。
- fake hardware 模式下，KDL 预览会用上一帧 `q_des` 作为下一次 FK/IK 的预览状态，避免 fake 关节不受力矩推进时 Meshcat 只能显示每周期一小步。真机模式仍使用 Franka 实际状态。
- 如果 KDL 对当前目标返回失败，控制器只限频打印 warning，并保持当前关节目标，不再抛异常退出 `ros2_control_node`。

额外加了三层安全限制：

- `max_target_linear_step`：每个控制周期朝目标最多走 `0.007 m`。
- `max_target_angular_step`：每个控制周期朝目标最多转 `0.03 rad`。
- `target_timeout_sec`：目标 pose 超时后保持当前关节位置，不继续追上一次目标。

末端 frame 必须一致。当前本地 fake 配置默认 `tcp_link_name: fr3_link8`，所以仿真链路中 `tracker_pose_target_node` 也保持默认 `ik_frame_name:=fr3_link8`。如果接真机并加载手爪 TCP，再把控制器和 tracker 目标节点一起切到 `fr3_hand_tcp`：

```text
load_gripper:=true
ik_frame_name:=fr3_hand_tcp
target_frame:=base
```

当前本地 fake 配置默认使用 `fr3_link8`，因为本地 `franka_description` 没有 `fr3_hand_tcp`。接真机或加载手爪 TCP 时，要让 `tracker_pose_target_node.ik_frame_name` 和控制器 `tcp_link_name` 保持一致。

## 构建

使用你的 conda Humble ROS 2 环境：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
colcon build --symlink-install \
  --base-paths "$FRANKA_ROOT" "$FRANKA_ROOT/franka_ros2" \
  --packages-select franka_description franka_tracker_bridge franka_arm_controllers
source install/setup.bash
```

构建 pose 型真机控制器需要完整 Franka 依赖，包括 `franka_msgs`、`franka_semantic_components`、`franka_hardware` 和 libfranka 开发包。依赖完整时：

当前 conda 环境已使用：

```bash
conda install -p "$CONDA_PREFIX" -c conda-forge libfranka=0.21.2
```

不要降到 `libfranka=0.15.0`，当前 `franka_hardware` 源码需要 `franka/async_control/async_position_control_handler.hpp`。

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
colcon build --symlink-install \
  --base-paths "$FRANKA_ROOT" "$FRANKA_ROOT/franka_ros2" \
  --packages-up-to franka_arm_controllers
source install/setup.bash
```

如果 CMake 在 conda 环境里找不到 Python/Numpy，可加：

```bash
--cmake-args \
  -DPython_EXECUTABLE=$CONDA_PREFIX/bin/python \
  -DPython_INCLUDE_DIRS=$CONDA_PREFIX/include/python3.12 \
  -DPython_LIBRARIES=$CONDA_PREFIX/lib/libpython3.12.so \
  -DPython_NumPy_INCLUDE_DIRS=$CONDA_PREFIX/lib/python3.12/site-packages/numpy/_core/include
```

## 启动 Meshcat 预览

单独启动这一条链路时，会看到 tracker 虚拟末端坐标轴；Franka 模型需要另一个终端启动 pose 控制器后，才会跟随 KDL JointState 运动：

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py
```

如果需要强制重新校准 tracker：

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py libsurvive_args:=--force-calibrate
```

如果有多个 tracker，可以指定序列号，只接其中一个：

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py target_serial:=LHR_XXXXXXXX
```

如果暂时不用 libsurvive 硬件输入，只想手动发 `/tracker/pose` 测试：

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py use_libsurvive:=false
```

启动后会自动尝试打开 Meshcat 网页，终端也会打印类似地址：

```text
Meshcat preview URL: http://127.0.0.1:7000/static/
```

如果浏览器没有自动弹出，手动打开这个地址，也能看到 Franka FR3 和 tracker 虚拟末端坐标轴。只有 pose 控制器也在运行时，Franka 才会跟随 `/franka_controller/kdl_desired_joint_states`。

如果不想自动打开浏览器：

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py open_browser:=false
```

启动前不要同时运行 `survive-websocketd` 或 `survive-cli`，否则可能抢占同一套 tracker 硬件。

## 启动 Meshcat + pose 控制器

终端 1：启动 tracker pose 目标和 Meshcat，当前本地 fake 链路使用 `fr3_link8`：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
source install/setup.bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py \
  libsurvive_args:=--force-calibrate \
  publish_deadman:=true \
  ik_frame_name:=fr3_link8
```

终端 2：启动 Franka 控制器链路，不启动 RViz，加载 pose 控制器：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
source install/setup.bash
ros2 launch franka_arm_controllers joint_impedance_pose_controller.launch.py \
  robot_config_file:=tracker_fake_fr3_config.yaml \
  controller_name:=joint_impedance_pose_controller
```

`tracker_fake_fr3_config.yaml` 只用于 fake hardware 启动链路检查。fake hardware 不保证完整模拟官方力矩阻抗控制器需要的 Franka model/state/collision service 行为。

目标真机是 Arm v2.1，接真机时使用专用配置：

```bash
ros2 launch franka_arm_controllers joint_impedance_pose_controller.launch.py \
  robot_config_file:=real_fr3v2_1_config.yaml \
  controller_name:=joint_impedance_pose_controller
```

tracker 侧必须同时使用 v2.1 模型和 frame。首次验证保持自动 deadman 关闭：

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py \
  config_file:="$FRANKA_ROOT/franka_ros2/install/franka_tracker_bridge/share/franka_tracker_bridge/config/tracker_bridge_fr3v2_1_real.yaml" \
  robot_type:=fr3v2_1 \
  ik_frame_name:=fr3v2_1_link8 \
  publish_deadman:=false
```

## 无 tracker 手动烟雾测试

另开一个终端：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
source install/setup.bash
```

打开 deadman：

```bash
ros2 topic pub /tracker/deadman std_msgs/msg/Bool "{data: true}"
```

发布第一帧 tracker pose 建立启动基准：

```bash
ros2 topic pub --once /tracker/pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: libsurvive_world},
  pose: {
    position: {x: 0.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

发布第二帧，让 tracker 相对 x 增加 `0.10 m`：

```bash
ros2 topic pub --once /tracker/pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: libsurvive_world},
  pose: {
    position: {x: 0.10, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

查看目标 pose：

```bash
ros2 topic echo /franka_controller/target_cartesian_pose
```

预期结果：如果 deadman 已打开，启动采样完成后会先输出由启动时当前关节角计算出的末端 pose；后续目标 x 按滤波后的 tracker 相对位移变化。手动单帧测试时可以临时用 `calibration_sample_count:=1 tracker_low_pass_alpha:=1.0 translation_deadband:=0.0 rotation_deadband:=0.0` 关闭启动采样和滤波。Meshcat 中 tracker 虚拟末端坐标轴会移动；如果 pose 控制器也在运行，FR3 会跟随 KDL IK 发布的 `/franka_controller/kdl_desired_joint_states`。

## 调参顺序

1. 先看 `/franka_controller/target_cartesian_pose`，确认第一帧基准和第二帧相对位移正确。
2. 调 `map_matrix`，确认 tracker 坐标轴和机器人目标坐标轴的正负号。
3. 调 `tracker_low_pass_alpha`、`translation_deadband`、`rotation_deadband` 和 `calibration_sample_count`，先把 tracker 抖动压住。
4. 调 `translation_scale` 和 `rotation_scale`，确认 tracker 运动幅度到机械臂目标幅度的比例。
5. 必要时再启用 `translation_limit` 或降低 `rotation_limit`，限制相对启动基准的目标幅度。
6. 看 Meshcat 中 tracker 虚拟末端坐标轴，确认方向、尺度和旋转手感符合直觉。
7. 再看 `/franka_controller/kdl_desired_joint_states` 和 Meshcat 中 Franka 是否跟随目标 pose 平滑运动；必要时先调控制器的 `max_target_linear_step`、`max_target_angular_step`，再调 tracker 的 scale/limit。

## 接真机前的现实边界

当前 `/franka_controller/target_cartesian_pose` 已经可以被新增的 `franka_arm_controllers/JointImpedancePoseController` 消费。旧的速度入口已删除。

真机前建议保持：

- deadman 默认关闭。
- 低速、低加速度、低幅度限制。
- 工作空间限制和奇异位形保护。
- 先空载、低速、小范围测试。
