# Franka FR3 Tracker 遥操作项目

Franka FR3 tracker 遥操作项目。

以下命令使用可覆盖的项目根目录变量，并默认使用 `franka` conda 环境：

```bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate franka
export FRANKA_ROOT="${FRANKA_ROOT:-$HOME/franka_description}"
```

- tracker/libsurvive 输入链路；
- Franka FR3 真机连接会用到的 bringup、hardware、message、semantic component、gripper；
- Meshcat + KDL IK 的算法验证链路；
- 自定义 pose 控制器和 tracker bridge 的真机接入链路。


## 保留的主要包

| 路径 | 作用 |
| --- | --- |
| `franka_tracker_bridge` | tracker 遥操作包。包含 `libsurvive_pose_node`、`tracker_pose_target_node`、`tracker_meshcat_preview_node`。 |
| `franka_bringup` | Franka 真机/fake hardware 的 ROS 2 bringup 入口。保留 FR3 arm 相关配置，移除 mobile/TMR 入口。 |
| `franka_hardware` | Franka ros2_control 硬件接口，后续连真机必须保留。 |
| `franka_semantic_components` | Franka state/model semantic interfaces，官方控制器依赖。 |
| `franka_robot_state_broadcaster` | 发布 Franka robot state，真机调试需要。 |
| `franka_msgs` | Franka ROS messages/services。 |
| `franka_gripper` | Franka Hand/夹爪支持。即使当前 Meshcat 不加载夹爪，真机或 hand TCP 场景可能需要。 |
| `franka_ros2` | 精简后的 meta package，依赖当前保留的 Franka + tracker 包。 |

详细节点和参数说明见：

- `docs/franka_tracker_teleop_readme.zh-CN.md`
- `docs/tracker_to_franka_control.zh-CN.md`
- `docs/franka_tracker_teleop_nodes_topics.zh-CN.md`
- `franka_tracker_bridge/docs/tracker_teleop_memory.md`

## 当前主链路

```text
libsurvive tracker
-> libsurvive_pose_node
-> /tracker/pose
-> tracker_pose_target_node
-> /franka_controller/target_cartesian_pose
-> joint_impedance_pose_controller  # franka_ros2/franka_arm_controllers
-> KDL IK
-> /franka_controller/kdl_desired_joint_states
-> tracker_meshcat_preview_node
-> Meshcat: Franka FR3 + tracker 虚拟末端坐标轴
```

`franka_tracker_bridge` 只负责 tracker 输入、相对位姿映射和 Meshcat 预览。真正的 pose 型 KDL/阻抗控制器在：

```text
$FRANKA_ROOT/franka_ros2/franka_arm_controllers
```

当前控制器名：

```text
joint_impedance_pose_controller
```


当前关键映射参数在 `franka_tracker_bridge/config/tracker_bridge_preview.yaml`：

```yaml
coord_swap: [1, 0, 2]
coord_flip: [1.0, -1.0, 1.0]
base_xy_rotation_deg: 90.0
orientation_alignment_rpy_deg: [0.0, 0.0, 180.0]
tracker_rotation_scale: -1.0
tracker_rotation_axis_scale: [1.0, 1.0, -1.0]
tracker_pos_soft_limit_mm: 15.0
tracker_pos_hard_limit_mm: 40.0
tracker_rot_soft_limit_deg: 6.0
tracker_rot_hard_limit_deg: 15.0
```

## 构建

使用你的 conda Humble ROS 2 环境：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
export LDFLAGS="-L$CONDA_PREFIX/lib ${LDFLAGS:-}"

colcon build --symlink-install \
  --base-paths "$FRANKA_ROOT" "$FRANKA_ROOT/franka_ros2" \
  --packages-select \
    franka_description \
    franka_msgs \
    franka_gripper \
    franka_hardware \
    franka_semantic_components \
    franka_robot_state_broadcaster \
    franka_bringup \
    franka_arm_controllers \
    franka_tracker_bridge \
    franka_ros2 \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DPython_EXECUTABLE=$CONDA_PREFIX/bin/python \
    -DPython_INCLUDE_DIRS=$CONDA_PREFIX/include/python3.12 \
    -DPython_LIBRARIES=$CONDA_PREFIX/lib/libpython3.12.so \
    -DPython_NumPy_INCLUDE_DIRS=$CONDA_PREFIX/lib/python3.12/site-packages/numpy/_core/include

source install/setup.bash
```


## Meshcat + KDL 仿真验证

终端 1：启动 Franka fake/controller 链路，加载 pose 控制器：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
source install/setup.bash

ros2 launch franka_arm_controllers joint_impedance_pose_controller.launch.py \
  robot_config_file:=tracker_fake_fr3_config.yaml \
  controller_name:=joint_impedance_pose_controller
```

终端 2：启动 tracker 输入、pose target 和 Meshcat：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
source install/setup.bash

ros2 launch franka_tracker_bridge tracker_preview.launch.py \
  libsurvive_args:=--force-calibrate \
  ik_frame_name:=fr3_link8 \
  publish_deadman:=true \
  open_browser:=true
```

启动后保持 tracker 静止约 3 秒，等待自动标定完成。Meshcat 网页会自动打开；如果没有弹出，终端会打印类似：

```text
http://127.0.0.1:7000/static/
```

不接 tracker，只验证 ROS 链路和 Meshcat：

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py \
  use_libsurvive:=false \
  open_browser:=true
```

## 常用检查命令

```bash
ros2 topic echo /tracker/pose
ros2 topic echo /tracker/deadman
ros2 topic echo /franka_controller/target_cartesian_pose
ros2 topic echo /franka_controller/kdl_desired_joint_states
ros2 control list_controllers
```

如果 Franka 模型不动，先看 `/franka_controller/kdl_desired_joint_states` 是否有数据。tracker 虚拟末端会动但机械臂不动，通常说明 pose 控制器没有启动、KDL 没有解出关节角，或 Meshcat 没收到 JointState。

## 真机接入注意

这台目标真机已由 Dashboard 确认为 `Arm3Rv2.1`、System Image `5.10.0`，C2 FCI
地址为 `172.16.0.2/24`。项目提供了不会覆盖 fake 默认配置的专用文件：

```text
franka_arm_controllers/config/real_fr3v2_1_config.yaml
franka_tracker_bridge/config/tracker_bridge_fr3v2_1_real.yaml
```

首次只启动控制器并读取状态，不启动 tracker 输入：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
source install/setup.bash

ros2 launch franka_arm_controllers joint_impedance_pose_controller.launch.py \
  robot_config_file:=real_fr3v2_1_config.yaml \
  controller_name:=joint_impedance_pose_controller
```

确认硬件和控制器状态正常后，再以禁用自动 deadman 的方式启动 v2.1 tracker 链路：

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py \
  config_file:="$FRANKA_ROOT/franka_ros2/install/franka_tracker_bridge/share/franka_tracker_bridge/config/tracker_bridge_fr3v2_1_real.yaml" \
  robot_type:=fr3v2_1 \
  ik_frame_name:=fr3v2_1_link8 \
  publish_deadman:=false \
  libsurvive_args:=--force-calibrate \
  open_browser:=true
```

真机 tracker 配置将位移和旋转缩放设为 `1.0`，并默认不自动发布 deadman。
在配置并验证实体 deadman 之前，不要使能运动目标。

真机链路仍然使用同一个 `/franka_controller/target_cartesian_pose` 输入，但控制器会输出真实关节阻抗力矩。接真机前至少确认：

- 使用 `real_fr3v2_1_config.yaml`，其中 `robot_ip=172.16.0.2`、`use_fake_hardware=false`。
- v2.1 的 `robot_type` 为 `fr3v2_1`，末端 frame 为 `fr3v2_1_link8`。
- `tracker_pose_target_node.ik_frame_name` 和控制器自动推导出的 `tcp_link_name` 必须完全一致。
- 首次真机测试时保持很小的 Tracker 位移，确认 deadman、超时保持、急停和通信稳定性可用。
- 确认 `/joint_states` 有完整的 7 关节实时数据；tracker 节点会锁存启动时的当前姿态，不需要预定位。
- 如果 KDL 打印 `did not converge`，控制器会保持当前关节目标，不应继续硬推 tracker 到奇异或不可达区域。
