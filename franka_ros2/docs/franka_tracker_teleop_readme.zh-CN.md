# Franka FR3 Tracker 遥操作 README

本文档说明当前 `franka_ros2` 单 workspace 中的 tracker 遥操作链路，包括依赖环境、项目文件、节点、话题、构建、fake 验证、真机接入和排错命令。

执行本文命令前先激活环境并设置项目根目录；项目移动后只需覆盖 `FRANKA_ROOT`：

```bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate franka
export FRANKA_ROOT="${FRANKA_ROOT:-$HOME/franka_description}"
```

## 1. 总体边界

Franka 真机不是通过 ROS topic 直接通信。ROS 2 运行在控制电脑上，真机通信由 `franka_hardware` 调用 `libfranka`，再通过 Franka Control Interface 和控制柜通信。

```text
tracker / 手动输入
-> franka_tracker_bridge
-> /franka_controller/target_cartesian_pose
-> franka_arm_controllers/JointImpedancePoseController
-> ros2_control command interfaces
-> franka_hardware
-> libfranka / FCI over Ethernet
-> Franka 控制柜
-> FR3 机械臂
```

遥操作节点只发布末端目标 pose。真正发给真机的是控制器在 1 kHz 控制循环里计算出的 7 维关节阻抗力矩命令。

## 2. Workspace

主 workspace：

```text
$FRANKA_ROOT/franka_ros2
```

当前已经把 pose 控制器合并到这个 workspace，正常使用时不再需要旧的外部 overlay。`$FRANKA_ROOT/franka_spacemouse` 已移除，不要再 source 或构建它。

常用 source：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
source install/setup.bash
```

确认包来自当前 workspace：

```bash
ros2 pkg prefix franka_arm_controllers
ros2 pkg prefix franka_tracker_bridge
ros2 pkg prefix franka_hardware
```

期望 `franka_arm_controllers` 指向：

```text
$FRANKA_ROOT/franka_ros2/install/franka_arm_controllers
```

## 3. 依赖环境

### 3.1 ROS 和 Python 环境

当前命令默认使用 conda 环境：

```text
$CONDA_PREFIX
```

需要的主要 ROS 2 能力：

```text
ROS 2 Humble
colcon
ament_cmake
ament_cmake_python
rclcpp
rclpy
ros2_control
controller_manager
joint_state_broadcaster
robot_state_publisher
xacro
```

Python 侧节点会用到：

```text
rclpy
ament_index_python
numpy
pinocchio
meshcat
xacro
```

C++ 真机和控制器侧会用到：

```text
libfranka
franka_msgs
franka_hardware
franka_semantic_components
controller_interface
hardware_interface
orocos_kdl
kdl_parser
moveit_msgs
pluginlib
Eigen3
TinyXML2
```

当前 Franka 依赖建议：

```bash
conda install -p "$CONDA_PREFIX" -c conda-forge libfranka=0.21.2
```

不要降到 `libfranka=0.15.0`，当前 `franka_hardware` 代码需要较新的 libfranka async control 头文件。

### 3.2 libsurvive

tracker 硬件输入依赖本地 libsurvive：

```text
$FRANKA_ROOT/libsurvive
```

`franka_tracker_bridge/CMakeLists.txt` 默认从这里找：

```text
include/libsurvive/survive_api.h
build/libsurvive.so
```

如果 libsurvive 没构建，`libsurvive_pose_node` 会编译失败。只想测试 ROS 链路时可用：

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py use_libsurvive:=false
```

### 3.3 真机运行环境

真机控制建议满足：

```text
控制电脑使用实时内核或低延迟配置
控制电脑网卡和 Franka FCI 口同网段
Franka Desk 中机器人已解锁并允许 FCI
急停、user stop、碰撞阈值、工作空间安全已确认
```

首次真机测试应降低 tracker 移动幅度，只做毫米级动作验证。

## 4. 主要项目文件

### 4.1 tracker 输入和目标生成

```text
franka_tracker_bridge/
  CMakeLists.txt
  package.xml
  launch/tracker_preview.launch.py
  config/tracker_bridge_preview.yaml
  src/libsurvive_pose_node.cpp
  include/franka_tracker_bridge/libsurvive_pose_conversions.hpp
  franka_tracker_bridge/tracker_pose_target_node.py
  franka_tracker_bridge/tracker_pose_target_core.py
  franka_tracker_bridge/tracker_meshcat_preview_node.py
  franka_tracker_bridge/tracker_meshcat_preview_core.py
  franka_tracker_bridge/pose_math.py
  franka_tracker_bridge/ros_conversions.py
```

作用：

```text
libsurvive_pose_node.cpp
  读取 tracker 位姿，发布 /tracker/pose 和 /tracker/deadman。

tracker_pose_target_node.py
  把 tracker 相对初始位姿映射成 Franka 末端目标 pose。

tracker_pose_target_core.py
  纯算法核心，处理标定、坐标变换、限幅、deadman。

tracker_meshcat_preview_node.py
  Meshcat 可视化节点，显示 FR3 模型和目标末端坐标轴。

tracker_bridge_preview.yaml
  tracker、目标 pose、Meshcat 的默认参数。
```

### 4.2 pose 控制器

```text
franka_arm_controllers/
  CMakeLists.txt
  package.xml
  launch/joint_impedance_pose_controller.launch.py
  launch/franka.launch.py
  urdf/franka_arm.urdf.xacro
  config/controllers.yaml
  config/tracker_fake_fr3_config.yaml
  config/real_fr3v2_1_config.yaml
  config/example_fr3_duo_config.yaml
  src/joint_impedance_pose_controller.cpp
  include/franka_arm_controllers/joint_impedance_pose_controller.hpp
```

作用：

```text
joint_impedance_pose_controller.cpp
  订阅 /franka_controller/target_cartesian_pose。
  用 KDL IK 计算 q_des。
  发布 /franka_controller/kdl_desired_joint_states 供 Meshcat 显示。
  根据 q_des、当前 q/dq 和 coriolis 输出关节阻抗力矩。

controllers.yaml
  注册 joint_impedance_pose_controller 并配置 KDL、限速、增益和话题。

tracker_fake_fr3_config.yaml
  fake hardware 链路测试配置。

real_fr3v2_1_config.yaml
  当前 Arm v2.1 真机配置，C2 FCI 地址为 172.16.0.2。
```

### 4.3 Franka 真机底层

```text
franka_hardware/
  src/franka_hardware_interface.cpp
  src/robot.cpp
  src/franka_param_service_server.cpp
  src/franka_action_server.cpp

franka_bringup/
  launch/franka.launch.py
  config/controllers.yaml

franka_robot_state_broadcaster/
franka_semantic_components/
franka_msgs/
```

作用：

```text
franka_hardware
  ros2_control 硬件插件，内部创建 franka::Robot(robot_ip)。

franka_bringup
  启动 robot_state_publisher、ros2_control_node、broadcasters。

franka_robot_state_broadcaster
  发布 Franka 完整状态和便捷状态 topic。

franka_semantic_components
  控制器读取 Franka state/model 的安全封装。

franka_msgs
  Franka msg、srv、action 定义。
```

## 5. 节点和控制器

### 5.1 `libsurvive_pose_node`

包：

```text
franka_tracker_bridge
```

可执行：

```text
libsurvive_pose_node
```

输入：

```text
libsurvive C API / tracker hardware
```

输出：

```text
/tracker/pose       geometry_msgs/msg/PoseStamped
/tracker/deadman    std_msgs/msg/Bool
```

关键参数：

```text
pose_topic          默认 /tracker/pose
deadman_topic       默认 /tracker/deadman
world_frame         默认 libsurvive_world
target_serial       指定 tracker 序列号，空字符串表示自动选择
lock_first_object   默认 true
ignore_lighthouses  默认 true
publish_deadman     默认 true
libsurvive_args     可传 --force-calibrate
working_directory   默认 $FRANKA_ROOT/libsurvive
```

### 5.2 `tracker_pose_target_node`

包：

```text
franka_tracker_bridge
```

可执行：

```text
tracker_pose_target_node
```

输入：

```text
/tracker/pose       geometry_msgs/msg/PoseStamped
/tracker/deadman    std_msgs/msg/Bool
/joy                sensor_msgs/msg/Joy，可选
```

输出：

```text
/franka_controller/target_cartesian_pose    geometry_msgs/msg/PoseStamped
```

核心行为：

```text
启动后先等待 /joint_states 的完整当前 7 关节状态。
通过 FK 计算并锁存 robot_start_pose；不使用固定关节角或固定末端位姿。
随后用 calibration_duration_sec 和 calibration_sample_count 建立 tracker_start。
后续计算 inverse(T_tracker_start) * T_tracker_current。
将 tracker 相对位姿经过坐标轴交换、符号翻转、旋转对齐、低通、死区、软硬限幅。
把相对位姿作用到 robot_start_pose，发布 Franka 在 base 坐标系下的绝对末端目标 pose。
```

重要安全点：

```text
没有有效启动关节状态时不处理 tracker pose，也不发布目标。
deadman 为 false 时不发布新目标 pose。
deadman 关闭再打开不会重新标定 tracker_start。
需要重新标定时重启 tracker_preview.launch.py。
```

关键参数：

```text
target_pose_topic              /franka_controller/target_cartesian_pose
target_frame                   base
joint_states_topic             /joint_states
robot_type                     fr3
load_gripper                   false
ik_frame_name                  fr3_link8
OLD
replace_once($teleop_doc, <<'OLD', <<'NEW');
| `/tracker/pose` | `geometry_msgs/msg/PoseStamped` | `libsurvive_pose_node` 或手动测试 | `tracker_pose_target_node` | tracker 原始位姿 |
OLD
| `/tracker/pose` | `geometry_msgs/msg/PoseStamped` | `libsurvive_pose_node` 或手动测试 | `tracker_pose_target_node` | tracker 原始位姿 |
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` | `tracker_pose_target_node` | 启动时锁存的当前 7 关节角 |
tracker_low_pass_alpha         0.25
translation_deadband           0.003
rotation_deadband              0.02
calibration_duration_sec       3.0
calibration_sample_count       30
coord_swap                     [1, 0, 2]
coord_flip                     [1.0, -1.0, 1.0]
base_xy_rotation_deg           90.0
orientation_alignment_rpy_deg  [0.0, 0.0, 180.0]
tracker_pos_soft_limit_mm      15.0
tracker_pos_hard_limit_mm      40.0
tracker_rot_soft_limit_deg     6.0
tracker_rot_hard_limit_deg     15.0
```

### 5.3 `tracker_meshcat_preview_node`

包：

```text
franka_tracker_bridge
```

可执行：

```text
tracker_meshcat_preview_node
```

输入：

```text
/franka_controller/target_cartesian_pose      geometry_msgs/msg/PoseStamped
/franka_controller/kdl_desired_joint_states   sensor_msgs/msg/JointState
```

输出：

```text
Meshcat browser view
```

默认行为：

```text
显示 FR3 模型。
显示 tracker 目标末端坐标轴。
默认不在 Meshcat 节点里做 IK。
使用 /franka_controller/kdl_desired_joint_states 驱动模型。
```

关键参数：

```text
target_pose_topic      /franka_controller/target_cartesian_pose
joint_states_topic     /franka_controller/kdl_desired_joint_states
robot_type             fr3
load_gripper           false
open_browser           true
enable_robot_ik        false
ik_frame_name          fr3_link8
publish_rate_hz        60.0
```

### 5.4 `joint_impedance_pose_controller`

包：

```text
franka_arm_controllers
```

插件类型：

```text
franka_arm_controllers/JointImpedancePoseController
```

它不是普通独立节点，而是由 `controller_manager` 加载到 `ros2_control_node` 中运行的 controller。

输入：

```text
/franka_controller/target_cartesian_pose    geometry_msgs/msg/PoseStamped
Franka state/model semantic interfaces
```

输出：

```text
/franka_controller/kdl_desired_joint_states sensor_msgs/msg/JointState
ros2_control effort command interfaces
```

核心行为：

```text
订阅目标末端 pose。
每个控制周期按 max_target_linear_step 和 max_target_angular_step 朝目标小步推进。
用 KDL ChainIkSolverPos_NR_JL 求 q_des。
IK 失败时限频 warning，并保持当前关节目标。
根据 Kp、Kd、当前关节状态和 coriolis 计算力矩。
把 7 维力矩写入 franka_hardware。
```

关键参数在：

```text
franka_arm_controllers/config/controllers.yaml
```

默认值：

```text
target_pose_topic                              /franka_controller/target_cartesian_pose
kdl_desired_joint_states_topic                 /franka_controller/kdl_desired_joint_states
ik_backend                                     kdl
base_link_name                                 base
tcp_link_name                                  fr3_link8
max_target_linear_step                         0.007
max_target_angular_step                        0.03
target_timeout_sec                             0.25
kdl_desired_joint_states_publish_rate_hz       60.0
ik_max_iterations                              1000
ik_eps                                         1.0e-3
fake_preview_follow_desired_state              true
```

### 5.5 `franka_hardware`

包：

```text
franka_hardware
```

插件：

```text
franka_hardware/FrankaHardwareInterface
```

作用：

```text
读取 robot_ip。
创建 libfranka::Robot(robot_ip)。
读取 robot state。
接收 ros2_control command interfaces。
通过 libfranka/FCI 与 Franka 控制柜通信。
```

它是接真机的关键组件，但不通过用户自定义 topic 接收目标。目标 topic 先进入 controller，再由 controller 写 command interface。

## 6. 话题和接口

### 6.1 遥操作主链路话题

| 话题 | 类型 | 发布者 | 订阅者 | 作用 |
| --- | --- | --- | --- | --- |
| `/tracker/pose` | `geometry_msgs/msg/PoseStamped` | `libsurvive_pose_node` 或手动测试 | `tracker_pose_target_node` | tracker 原始位姿 |
| `/tracker/deadman` | `std_msgs/msg/Bool` | `libsurvive_pose_node` 或手动测试 | `tracker_pose_target_node` | 安全开关 |
| `/joy` | `sensor_msgs/msg/Joy` | 可选手柄节点 | `tracker_pose_target_node` | 可选 deadman 按钮 |
| `/franka_controller/target_cartesian_pose` | `geometry_msgs/msg/PoseStamped` | `tracker_pose_target_node` | `joint_impedance_pose_controller`、`tracker_meshcat_preview_node` | Franka 末端目标 pose |
| `/franka_controller/kdl_desired_joint_states` | `sensor_msgs/msg/JointState` | `joint_impedance_pose_controller` | `tracker_meshcat_preview_node` | KDL IK 解出的期望关节角 |

### 6.2 真机状态话题

常用状态 topic：

```text
/joint_states
/franka_robot_state_broadcaster/robot_state
/franka_robot_state_broadcaster/current_pose
/franka_robot_state_broadcaster/measured_joint_states
/franka_robot_state_broadcaster/desired_joint_states
/franka_robot_state_broadcaster/external_joint_torques
/franka_robot_state_broadcaster/external_wrench_in_base_frame
/franka_robot_state_broadcaster/external_wrench_in_stiffness_frame
```

便捷状态 topic 通常使用 best effort QoS。`ros2 topic echo` 需要时可加：

```bash
ros2 topic echo --qos-reliability best_effort /franka_robot_state_broadcaster/current_pose
```

### 6.3 真机服务和 action

非实时参数服务由 `franka_hardware` 提供，常见服务：

```text
/service_server/set_joint_stiffness
/service_server/set_cartesian_stiffness
/service_server/set_tcp_frame
/service_server/set_stiffness_frame
/service_server/set_force_torque_collision_behavior
/service_server/set_full_collision_behavior
/service_server/set_load
```

错误恢复 action：

```text
/action_server/error_recovery
```

调用：

```bash
ros2 action send_goal /action_server/error_recovery franka_msgs/action/ErrorRecovery {}
```

## 7. 构建

进入 workspace：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
```

可选环境变量：

```bash
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
export LDFLAGS="-L$CONDA_PREFIX/lib ${LDFLAGS:-}"
```

完整构建 tracker、pose 控制器和 Franka 真机依赖：

```bash
colcon build --symlink-install --base-paths "$FRANKA_ROOT" "$FRANKA_ROOT/franka_ros2" --packages-select franka_description franka_msgs franka_gripper franka_hardware franka_semantic_components franka_robot_state_broadcaster franka_bringup franka_arm_controllers franka_tracker_bridge franka_ros2 --cmake-args -DBUILD_TESTING=OFF -DPython_EXECUTABLE=$CONDA_PREFIX/bin/python -DPython_INCLUDE_DIRS=$CONDA_PREFIX/include/python3.12 -DPython_LIBRARIES=$CONDA_PREFIX/lib/libpython3.12.so -DPython_NumPy_INCLUDE_DIRS=$CONDA_PREFIX/lib/python3.12/site-packages/numpy/_core/include
```

构建完成后：

```bash
source install/setup.bash
```

只构建控制器及依赖：

```bash
colcon build --symlink-install --base-paths "$FRANKA_ROOT" "$FRANKA_ROOT/franka_ros2" --packages-up-to franka_arm_controllers --cmake-args -DBUILD_TESTING=OFF
```

只构建 tracker bridge：

```bash
colcon build --symlink-install --base-paths "$FRANKA_ROOT" "$FRANKA_ROOT/franka_ros2" --packages-select franka_tracker_bridge --cmake-args -DBUILD_TESTING=OFF
```

## 8. Fake hardware 验证

终端 1，启动 fake Franka 控制器链路：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
source install/setup.bash
ros2 launch franka_arm_controllers joint_impedance_pose_controller.launch.py robot_config_file:=tracker_fake_fr3_config.yaml controller_name:=joint_impedance_pose_controller
```

终端 2，启动 tracker 目标生成和 Meshcat：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
source install/setup.bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py libsurvive_args:=--force-calibrate ik_frame_name:=fr3_link8 publish_deadman:=true open_browser:=true debug_curves:=true
```

如果没有 tracker 硬件：

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py use_libsurvive:=false open_browser:=true debug_curves:=true
```

此时可以手动注入 deadman 和 tracker pose：

```bash
ros2 topic pub /tracker/deadman std_msgs/msg/Bool "{data: true}"
```

```bash
ros2 topic pub --once /tracker/pose geometry_msgs/msg/PoseStamped "{header: {frame_id: libsurvive_world}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

再发布一个小位移：

```bash
ros2 topic pub --once /tracker/pose geometry_msgs/msg/PoseStamped "{header: {frame_id: libsurvive_world}, pose: {position: {x: 0.02, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

检查输出：

```bash
ros2 topic echo /franka_controller/target_cartesian_pose
ros2 topic echo /franka_controller/kdl_desired_joint_states
ros2 control list_controllers
```

## 9. 真机接入

### 9.1 修改真机配置

配置文件：

```text
franka_arm_controllers/config/real_fr3v2_1_config.yaml
```

该配置与 Dashboard 中的 Arm v2.1 和 C2 地址对应：

```yaml
robot1:
  arm_id: "fr3v2_1"
  arm_prefix: ""
  robot_ip: "172.16.0.2"
  use_fake_hardware: "false"
  load_gripper: "false"
  urdf_file: "urdf/franka_arm.urdf.xacro"
  arm_mounting_orientation: [0,0,0]
```

`robot_ip` 必须是控制电脑能 ping 到的 Franka FCI IP。

### 9.2 网络和 Desk 检查

```bash
ping <你的 Franka FCI IP>
```

Franka Desk 中确认：

```text
机器人已解锁
没有 error
FCI 可用
急停和 user stop 可用
机械臂附近安全
```

### 9.3 启动真机控制器链路

终端 1：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
source install/setup.bash
ros2 launch franka_arm_controllers joint_impedance_pose_controller.launch.py robot_config_file:=real_fr3v2_1_config.yaml controller_name:=joint_impedance_pose_controller
```

检查：

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo --once /joint_states
ros2 topic info -v /franka_controller/target_cartesian_pose
```

期望：

```text
FrankaHardwareInterface active
joint_state_broadcaster active
franka_robot_state_broadcaster active
joint_impedance_pose_controller active
/franka_controller/target_cartesian_pose 有 joint_impedance_pose_controller 订阅者
```

### 9.4 启动 tracker 遥操作

终端 2：

```bash
cd "$FRANKA_ROOT/franka_ros2"
source "$CONDA_PREFIX/setup.bash"
source install/setup.bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py \
  config_file:="$FRANKA_ROOT/franka_ros2/install/franka_tracker_bridge/share/franka_tracker_bridge/config/tracker_bridge_fr3v2_1_real.yaml" \
  robot_type:=fr3v2_1 \
  ik_frame_name:=fr3v2_1_link8 \
  publish_deadman:=false \
  libsurvive_args:=--force-calibrate \
  open_browser:=true
```

启动后保持 tracker 静止约 3 秒，等待自动标定完成。第一次测试只做很小位移，并确认 deadman、急停和 user stop 都可用。

### 9.5 手爪或 TCP 变化

当前默认 TCP：

```text
fr3v2_1_link8
```

如果接真机时使用 hand TCP，需要同时改两处，保持完全一致：

```text
tracker_pose_target_node.ik_frame_name
joint_impedance_pose_controller.tcp_link_name
```

示例 tracker 启动：

```bash
ros2 launch franka_tracker_bridge tracker_preview.launch.py libsurvive_args:=--force-calibrate load_gripper:=true ik_frame_name:=fr3_hand_tcp target_frame:=base
```

控制器的 `tcp_link_name` 在：

```text
franka_arm_controllers/config/controllers.yaml
```

## 10. 常用检查命令

查看节点：

```bash
ros2 node list
```

查看 tracker 输入：

```bash
ros2 topic echo /tracker/pose
ros2 topic echo /tracker/deadman
```

查看目标 pose：

```bash
ros2 topic echo /franka_controller/target_cartesian_pose
ros2 topic info -v /franka_controller/target_cartesian_pose
```

查看 KDL 输出：

```bash
ros2 topic echo /franka_controller/kdl_desired_joint_states
```

查看控制器：

```bash
ros2 control list_controllers
ros2 control list_hardware_components
```

查看 Franka 状态：

```bash
ros2 topic echo --once /joint_states
ros2 topic echo --once /franka_robot_state_broadcaster/robot_state
```

错误恢复：

```bash
ros2 action send_goal /action_server/error_recovery franka_msgs/action/ErrorRecovery {}
```

Humble 下硬件错误后，必要时先停 controller，再恢复：

```bash
ros2 control switch_controllers --deactivate joint_impedance_pose_controller
ros2 action send_goal /action_server/error_recovery franka_msgs/action/ErrorRecovery {}
ros2 control set_hardware_component_state FrankaHardwareInterface active
ros2 control switch_controllers --activate joint_impedance_pose_controller
```

## 11. 排错

### 11.1 `/franka_controller/target_cartesian_pose` 没有消息

检查：

```bash
ros2 topic echo /tracker/pose
ros2 topic echo /tracker/deadman
```

常见原因：

```text
tracker 没有数据
/tracker/deadman 为 false
还在启动标定阶段
tracker_pose_target_node 没启动
```

### 11.2 Meshcat 里目标轴动，机械臂模型不动

检查：

```bash
ros2 topic echo /franka_controller/kdl_desired_joint_states
ros2 control list_controllers
```

常见原因：

```text
joint_impedance_pose_controller 没有启动
KDL IK 没有解出 q_des
Meshcat 没订阅到 /franka_controller/kdl_desired_joint_states
```

### 11.3 控制器启动失败

检查：

```bash
ros2 pkg prefix franka_arm_controllers
ros2 launch franka_arm_controllers joint_impedance_pose_controller.launch.py --show-args
```

常见原因：

```text
没有 source install/setup.bash
robot_config_file 名字写错
real_fr3v2_1_config.yaml 中 robot_ip 错误
同名包被旧 workspace overlay 覆盖
```

### 11.4 真机不连接

检查：

```bash
ping <你的 Franka FCI IP>
ros2 control list_hardware_components
```

常见原因：

```text
网卡不在同网段
Franka Desk 未允许 FCI
机器人有 error 未恢复
robot_ip 不是 FCI IP
libfranka 版本或运行时库路径不对
```

### 11.5 KDL 打印 did not converge

含义：目标 pose 当前不可达、接近奇异、或目标跳变太大。当前控制器会保持当前关节目标，不会让 `ros2_control_node` 退出。

处理：

```text
减小 tracker 位移
降低 tracker_pos_hard_limit_mm 和 tracker_rot_hard_limit_deg
检查 ik_frame_name 和 tcp_link_name 是否一致
确认 /joint_states 已被目标节点完整锁存，且当前姿态远离奇异位形和工作空间边界
```

## 12. 推荐真机首测流程

1. 先只启动真机控制器链路，不启动 tracker，并确认 `joint_impedance_pose_controller` 为 active。
2. 确认 `/joint_states` 和 `franka_robot_state_broadcaster/robot_state` 有实时数据。
3. 让机械臂停在任意安全、可达且远离奇异的当前姿态；不需要移动到预设 q。
4. 启动 `tracker_preview.launch.py`，保持机械臂和 tracker 静止；确认日志出现已锁存当前 7 关节角。
5. 等待约 3 秒 tracker 自动标定完成，确认首个 `/franka_controller/target_cartesian_pose` 与启动时当前末端位姿一致。
6. 小幅移动 tracker，观察 Meshcat 和 `/franka_controller/kdl_desired_joint_states`。
7. 再让真机跟随毫米级动作。
8. 测试 deadman 关闭后目标是否停止更新。
9. 测试 user stop 和急停可用。

## 13. 关键结论

```text
接真机靠 robot_ip 和 franka_hardware，不靠额外的接真机 topic。
遥操作节点发布 /franka_controller/target_cartesian_pose。
joint_impedance_pose_controller 订阅该 topic。
控制器算 IK 和阻抗力矩。
franka_hardware 通过 libfranka/FCI 把命令发给控制柜。
```
