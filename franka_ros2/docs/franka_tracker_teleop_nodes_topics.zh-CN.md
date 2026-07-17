# Franka Tracker 遥操作节点与话题清单

本文档整理当前仓库中 Franka tracker 遥操作预览链路的节点、话题、参数和验证命令。当前默认链路用于 Meshcat 算法验证，不向真机发送关节力矩或速度命令。
新增的 pose 型真机控制器入口已经放在 `franka_arm_controllers` 中，真机链路可直接消费 `/franka_controller/target_cartesian_pose`。

执行本文命令前先激活环境并设置项目根目录：

```bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate franka
export FRANKA_ROOT="${FRANKA_ROOT:-$HOME/franka_description}"
```

## 当前默认链路

有 tracker 硬件时：

```text
libsurvive tracker
-> libsurvive_pose_node
-> /tracker/pose
-> tracker_pose_target_node
-> /franka_controller/target_cartesian_pose
-> joint_impedance_pose_controller
-> /franka_controller/kdl_desired_joint_states
-> tracker_meshcat_preview_node
-> Meshcat: Franka FR3 + tracker 虚拟末端坐标轴
```

无 tracker 硬件时：

```text
手动发布 /tracker/deadman + /tracker/pose
-> tracker_pose_target_node
-> /franka_controller/target_cartesian_pose
-> joint_impedance_pose_controller
-> /franka_controller/kdl_desired_joint_states
-> tracker_meshcat_preview_node
-> Meshcat
```

真机 pose 控制链路：

```text
libsurvive tracker
-> libsurvive_pose_node
-> /tracker/pose
-> tracker_pose_target_node
-> /franka_controller/target_cartesian_pose
-> franka_arm_controllers/JointImpedancePoseController
-> 官方 KDL IK
-> 关节阻抗力矩
-> Franka FR3
```

## 启动命令

构建：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
colcon build --symlink-install \
  --base-paths "$FRANKA_ROOT" "$FRANKA_ROOT/franka_ros2" \
  --packages-up-to franka_bringup franka_arm_controllers franka_tracker_bridge franka_ros2 \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

连接 tracker 时：

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py libsurvive_args:=--force-calibrate
```

不连接 tracker，只验证链路时：

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py use_libsurvive:=false
```

Meshcat 地址会在终端打印，默认通常是：

```text
http://127.0.0.1:7000/static/
```

## 节点

| 节点 | 可执行文件 | 作用 |
| --- | --- | --- |
| `libsurvive_pose_node` | `libsurvive_pose_node` | 调用 libsurvive C API，读取 tracker 位姿，发布 `/tracker/pose` 和 `/tracker/deadman`。 |
| `tracker_pose_target_node` | `tracker_pose_target_node` | 把 tracker 相对位姿变化映射成 Franka 末端目标 pose，发布 `/franka_controller/target_cartesian_pose`。 |
| `tracker_meshcat_preview_node` | `tracker_meshcat_preview_node` | 默认订阅目标 pose 和 KDL 期望关节 JointState；目标 pose 用来显示 tracker 虚拟末端坐标轴，JointState 用来驱动 Meshcat 中的 FR3 模型。 |
| `joint_impedance_pose_controller` | `franka_arm_controllers/JointImpedancePoseController` | 真机/fake controller，订阅目标 pose，用官方 KDL IK 解期望关节角，发布 `/franka_controller/kdl_desired_joint_states`，并输出关节阻抗力矩。 |
| Meshcat server | `meshcat.servers.zmqserver` | Meshcat 自动启动的 Web 可视化服务，不是 ROS 节点。 |

## 话题

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
| --- | --- | --- | --- | --- |
| `/tracker/pose` | `geometry_msgs/msg/PoseStamped` | `libsurvive_pose_node` 或手动测试脚本 | `tracker_pose_target_node` | tracker 原始空间位姿输入。 |
| `/tracker/deadman` | `std_msgs/msg/Bool` | `libsurvive_pose_node` 或手动测试脚本 | `tracker_pose_target_node` | 安全开关，`true` 时允许更新目标 pose。 |
| `/franka_controller/target_cartesian_pose` | `geometry_msgs/msg/PoseStamped` | `tracker_pose_target_node` | `tracker_meshcat_preview_node`，`joint_impedance_pose_controller` | 当前主链路的 Franka 末端目标 pose。 |
| `/franka_controller/kdl_desired_joint_states` | `sensor_msgs/msg/JointState` | `joint_impedance_pose_controller` | `tracker_meshcat_preview_node` | KDL IK 解出的期望 7 关节角；Meshcat 默认用这个话题显示 Franka。 |
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` | `tracker_pose_target_node` | 启动时机械臂实测关节角；目标节点只锁存首个完整有效样本作为 FK 基准。 |
| `/joy` | `sensor_msgs/msg/Joy` | 可选手柄节点 | `tracker_pose_target_node` | 可选 deadman 按钮输入；默认 `deadman_joy_button: -1`，未启用。 |

## 关键参数

配置文件：

```text
franka_tracker_bridge/config/tracker_bridge_preview.yaml
```

### `libsurvive_pose_node`

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `pose_topic` | `/tracker/pose` | tracker 位姿输出话题。 |
| `deadman_topic` | `/tracker/deadman` | deadman 输出话题。 |
| `target_serial` | `""` | 空字符串表示锁定第一个非 lighthouse tracker。 |
| `ignore_lighthouses` | `true` | 不把 lighthouse 当 tracker 发布。 |
| `publish_deadman` | `true` | tracker pose 到来时发布 deadman。 |
| `libsurvive_args` | `""` | 可通过 launch 传 `--force-calibrate`。 |
| `working_directory` | `$FRANKA_ROOT/libsurvive` | libsurvive 工作目录。 |

### `tracker_pose_target_node`

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `pose_topic` | `/tracker/pose` | tracker 原始位姿输入。 |
| `target_pose_topic` | `/franka_controller/target_cartesian_pose` | Franka 末端目标 pose 输出。 |
| `deadman_initially_enabled` | `false` | 默认不允许运动；需要收到 `/tracker/deadman=true`。 |
| `joint_states_topic` | `/joint_states` | 启动关节状态输入；必须包含完整、有限且在限位内的 7 个臂关节。 |
| `ik_frame_name` | `fr3_link8` | 作为末端目标的 Franka frame。FR3 仍然只有 7 个主动关节。 |
| `translation_scale` | `[1.0, 1.0, 1.0]` | tracker 平移相对量到机器人目标平移的比例。 |
| `rotation_scale` | `[1.0, 1.0, 1.0]` | tracker 旋转相对量到机器人目标旋转的比例。 |
| `translation_limit` | `[-1.0, -1.0, -1.0]` | tracker 相对启动标定点的最大映射平移；负数表示该轴不做人为限幅。 |
| `rotation_limit` | `3.141592653589793` | tracker 相对启动标定姿态的最大映射旋转，单位 rad；对四元数最短相对旋转等价于不额外限幅。 |
| `tracker_low_pass_alpha` | `0.25` | tracker 原始 pose 低通滤波系数；越小越稳但延迟越大，`1.0` 表示不滤波。 |
| `translation_deadband` | `0.003` | tracker 相对平移死区，单位 m，小于该值的轴向抖动置 0。 |
| `rotation_deadband` | `0.02` | tracker 相对旋转死区，单位 rad，小于该值的轴向抖动置 0。 |
| `calibration_sample_count` | `30` | 启动后先接收并滤波多少帧 tracker pose，再记录 `tracker_start`。 |
| `map_matrix` | 单位矩阵 | tracker 坐标到机器人控制坐标的轴映射和符号映射。 |

当前 pose 型映射公式：

```text
节点启动时锁存 /joint_states 的当前 7 关节角，通过 FK 计算 fr3_link8 的 robot_start_pose；没有固定初始姿态
节点启动后先对 tracker pose 做低通滤波
默认前 calibration_duration_sec=3.0 秒做自动标定，记录这段时间的 tracker 平均位姿作为 tracker_start
deadman 只控制是否输出目标 pose；不改变 tracker_start

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

robot_dp_local = clamp_axis(
  R_map * tracker_dp_local * translation_scale,
  translation_limit
)
robot_rotation_vector = clamp_norm(
  R_map * rotation_vector * rotation_scale,
  rotation_limit
)

candidate_pose.position = p_rs + R_rs * robot_dp_local
candidate_pose.orientation = R_rs * exp(robot_rotation_vector)

robot target = candidate_pose
publish candidate_pose
```

节点收到有效 `/joint_states` 前不会处理 tracker pose 或发布目标。锁存当前关节 FK 后，前 `calibration_sample_count` 帧只用于滤波和建立 tracker 基准，不会产生位移；如果建立基准时 deadman 为 `false`，节点不会发布目标 pose，但仍会记录 tracker 启动基准。后续对齐 DexCap：用 `T_tracker_start^-1 * T_tracker_current` 得到 tracker 相对初始 SE(3)，再乘到 `T_robot_start` 上生成目标 pose。
deadman 关闭期间不会重置 `tracker_start`；再次打开后会直接输出当前 tracker 相对启动基准对应的目标 pose。当前节点不做“无解点/工作空间超界”预过滤；它会持续发布相对启动基准生成的目标 pose，Meshcat IK 或后续真机控制层再处理能否跟随。
注意：这里的位姿变化量按 SE(3) 组合计算，不是把 tracker 的绝对位置直接发给机械臂，也不是简单把世界坐标位置分量相减后加到末端。

### `tracker_meshcat_preview_node`

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `target_pose_topic` | `/franka_controller/target_cartesian_pose` | 当前主链路输入。 |
| `joint_states_topic` | `/franka_controller/kdl_desired_joint_states` | Meshcat 中 Franka 模型的关节角输入。 |
| `robot_type` | `fr3` | 加载 FR3 模型。 |
| `load_gripper` | `false` | 当前不加载夹爪。 |
| `open_browser` | `true` | 启动 Meshcat 后自动尝试打开浏览器网页。 |
| `enable_robot_ik` | `false` | 默认不在 Meshcat 节点里做 Pinocchio IK，而是显示 KDL JointState。 |
| `ik_frame_name` | `fr3_link8` | 仅在重新启用 Meshcat 内部 IK 时使用。 |
| `ik_max_iterations` | `8` | 每次更新最多 IK 迭代次数。 |
| `ik_dt` | `0.35` | IK 单步积分系数。 |
| `max_joint_step` | `0.04` | Meshcat 每次更新最大关节变化，单位 rad。 |

### `joint_impedance_pose_controller`

配置文件：

```text
$FRANKA_ROOT/franka_ros2/franka_arm_controllers/config/controllers.yaml
```

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `target_pose_topic` | `/franka_controller/target_cartesian_pose` | tracker pose 目标输入。 |
| `kdl_desired_joint_states_topic` | `/franka_controller/kdl_desired_joint_states` | KDL IK 期望关节角调试输出，供 Meshcat 显示。 |
| `base_link_name` | `base` | KDL chain 根 link。 |
| `tcp_link_name` | `fr3_link8` | KDL IK 目标末端 frame。必须和 `tracker_pose_target_node.ik_frame_name` 一致。 |
| `max_target_linear_step` | `0.007` | 每个控制周期朝目标最多移动距离，单位 m。 |
| `max_target_angular_step` | `0.03` | 每个控制周期朝目标最多旋转角度，单位 rad。 |
| `target_timeout_sec` | `0.25` | 超过该时间没有新目标 pose 后，保持当前关节位置。 |
| `kdl_desired_joint_states_publish_rate_hz` | `60.0` | KDL 期望关节角调试话题发布频率。 |
| `k_gains` | 7 个数 | 官方关节阻抗刚度参数。 |
| `d_gains` | 7 个数 | 官方关节阻抗阻尼参数。 |

关键实现：

```text
收到 /franka_controller/target_cartesian_pose
-> 记录 target_position / target_orientation
-> update 中读取当前 Franka EE pose 和当前 7 关节
-> 每周期按 max_target_linear_step / max_target_angular_step 朝 target pose 前进一步
-> KDL ChainIkSolverPos_NR_JL 求 q_des
-> 发布 /franka_controller/kdl_desired_joint_states
-> tau = Kp * (q_des - q) - Kd * dq_filtered + coriolis
```

pose 是控制器输入。每周期的小步限制只是控制器内部安全推进，避免 tracker 目标跳变时直接给 KDL 一个大跨度目标。
在 fake hardware 模式下，控制器会用上一帧 KDL `q_des` 作为下一次预览 FK/IK 状态，让 Meshcat 能看到连续跟随；真机模式仍以 Franka 实际关节和末端状态为准。
如果 KDL 对当前目标返回失败，控制器只限频打印 warning，并保持当前关节目标，不会退出 `ros2_control_node`。

## 和真机 controller 的关系

当前 Meshcat 链路验证 tracker 相对位姿算法和 KDL IK 输出：

```text
/franka_controller/target_cartesian_pose
-> joint_impedance_pose_controller
-> /franka_controller/kdl_desired_joint_states
-> tracker_meshcat_preview_node
-> Meshcat 显示
```

`/franka_controller/target_cartesian_pose` 现在有两类消费者：

- Meshcat 预览：`tracker_meshcat_preview_node`，显示 tracker 目标坐标轴，并用 KDL JointState 显示 FR3 姿态。
- 真机/fake 控制：`joint_impedance_pose_controller`，用官方 KDL IK 和关节阻抗力矩。

tracker pose 链路应加载 `joint_impedance_pose_controller`。

控制链路末端 frame 必须一致。当前本地 fake 配置默认 `tcp_link_name: fr3_link8`，启动 tracker pose 目标时保持默认即可。接真机并加载手爪 TCP 时，再把 pose 控制器和 tracker pose 目标一起切到 `fr3_hand_tcp`：

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py \
  libsurvive_args:=--force-calibrate \
  load_gripper:=true \
  ik_frame_name:=fr3_hand_tcp \
  target_frame:=base
```

控制器启动：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
source install/setup.bash
ros2 launch franka_arm_controllers joint_impedance_pose_controller.launch.py \
  robot_config_file:=tracker_fake_fr3_config.yaml \
  controller_name:=joint_impedance_pose_controller
```

接真机时把 `robot_config_file` 换成真机 IP 配置，例如 `example_fr3_config.yaml`。真机前必须保留低速限制、幅度限制、deadman 和小范围测试。

## 验证命令

查看节点：

```bash
ros2 node list | grep tracker
```

默认应看到：

```text
/libsurvive_pose_node        # use_libsurvive:=true 时
/tracker_pose_target_node
/tracker_meshcat_preview_node
```

查看目标 pose 连接：

```bash
ros2 topic info /franka_controller/target_cartesian_pose
```

期望至少看到：

```text
Publisher count: 1
Subscription count: 1
```

查看目标 pose 输出：

```bash
ros2 topic echo /franka_controller/target_cartesian_pose
```

无 tracker 手动注入 deadman：

```bash
ros2 topic pub /tracker/deadman std_msgs/msg/Bool "{data: true}"
```

无 tracker 手动发布第一帧 pose 建立启动基准：

```bash
ros2 topic pub --once /tracker/pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: libsurvive_world},
  pose: {
    position: {x: 0.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

再发布第二帧产生相对位移：

```bash
ros2 topic pub --once /tracker/pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: libsurvive_world},
  pose: {
    position: {x: 0.10, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

这时 `/franka_controller/target_cartesian_pose` 的 x 应该相对启动时当前末端 x 增加约 `0.10 m`，Meshcat 中 tracker 虚拟末端坐标轴和 FR3 末端也应跟随变化。

## 常见现象

- 第一帧 tracker pose 只设置基准，目标 pose 会等于启动时 FR3 末端 pose。
- `/tracker/deadman` 为 `false` 时，`tracker_pose_target_node` 不发布目标 pose。
- deadman 关闭再打开不会重置启动标定；重新打开后会按当前 tracker 相对启动基准生成目标 pose。
- 启动抖动优先调 `calibration_sample_count`、`tracker_low_pass_alpha`、`translation_deadband` 和 `rotation_deadband`。
- 目标移动小，通常是 `translation_scale` 太小、`translation_limit` 太小、`max_joint_step` 太小，或 IK 接近工作空间边界。
- `fr3_link8` 是末端 frame，不是第 8 个主动关节；FR3 主动关节仍是 7 个。
- `sequence size exceeds remaining buffer` 是当前 conda ROS 环境里常见的 DDS/日志噪声；如果话题能正常发布订阅，可先忽略。
