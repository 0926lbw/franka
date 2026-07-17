franka_bringup
==============

Installation
------------

Please refer to the workspace README for build and launch commands.

Package Overview
----------------

This trimmed workspace keeps the core ``franka.launch.py`` entry point for Franka FR3 hardware and fake hardware bringup.
The launch file starts robot description publishing, ros2_control, joint state publishing, the Franka robot state broadcaster, and the optional Franka gripper.

Start the core robot bringup with::

    ros2 launch franka_bringup franka.launch.py robot_type:=fr3 robot_ip:=FCI_IP use_fake_hardware:=false

For tracker teleoperation, load the custom pose controller from ``franka_arm_controllers``:

.. code-block:: shell

    ros2 control load_controller --set-state active joint_impedance_pose_controller

The controller consumes ``/franka_controller/target_cartesian_pose`` and publishes KDL preview joint states on ``/franka_controller/kdl_desired_joint_states``.

Non-realtime robot parameter setting
------------------------------------

Non-realtime robot parameter setting can be done via ROS 2 services. They are advertised after the robot hardware is initialized.

Service names are given below::

 * /service_server/set_cartesian_stiffness
 * /service_server/set_force_torque_collision_behavior
 * /service_server/set_full_collision_behavior
 * /service_server/set_joint_stiffness
 * /service_server/set_load
 * /service_server/set_parameters
 * /service_server/set_parameters_atomically
 * /service_server/set_stiffness_frame
 * /service_server/set_tcp_frame

Service message descriptions are given below.

 * ``franka_msgs::srv::SetJointStiffness`` specifies joint stiffness for the internal controller
   (damping is automatically derived from the stiffness).
 * ``franka_msgs::srv::SetCartesianStiffness`` specifies Cartesian stiffness for the internal
   controller (damping is automatically derived from the stiffness).
 * ``franka_msgs::srv::SetTCPFrame`` specifies the transformation from <robot_type>_EE (end effector) to
   <robot_type>_NE (nominal end effector) frame. The transformation from flange to end effector frame
   is split into two transformations: <robot_type>_EE to <robot_type>_NE frame and <robot_type>_NE to
   <robot_type>_link8 frame. The transformation from <robot_type>_NE to <robot_type>_link8 frame can only be
   set through the administrator's interface.
 * ``franka_msgs::srv::SetStiffnessFrame`` specifies the transformation from <robot_type>_K to <robot_type>_EE frame.
 * ``franka_msgs::srv::SetForceTorqueCollisionBehavior`` sets thresholds for external Cartesian
   wrenches to configure the collision reflex.
 * ``franka_msgs::srv::SetFullCollisionBehavior`` sets thresholds for external forces on Cartesian
   and joint level to configure the collision reflex.
 * ``franka_msgs::srv::SetLoad`` sets an external load to compensate (e.g. of a grasped object).

Launch franka_bringup/franka.launch.py file to initialize robot hardware::

    ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip>

Here is a minimal example:

.. code-block:: shell

    ros2 service call /service_server/set_joint_stiffness \
      franka_msgs/srv/SetJointStiffness \
      "{joint_stiffness: [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]}"

.. important::

    Non-realtime parameter setting can only be done when the robot hardware is in `idle` mode.
    If a controller is active and claims command interface this will put the robot in the `move` mode.
    In `move` mode non-realtime param setting is not possible.

.. important::

    The <robot_type>_EE frame denotes the part of the
    configurable end effector frame which can be adjusted during run time through `franka_ros`. The
    <robot_type>_K frame marks the center of the internal
    Cartesian impedance. It also serves as a reference frame for external wrenches. *Neither the
    <robot_type>_EE nor the <robot_type>_K are contained in the URDF as they can be changed at run time*.
    By default, <robot_type> is set to "panda".

    .. figure:: ../../docs/assets/frames.svg
        :align: center
        :figclass: align-center

        Overview of the end-effector frames.

Non-realtime ROS 2 actions
--------------------------

Non-realtime ROS 2 actions can be done via the `ActionServer`. Following actions are available:

* ``/action_server/error_recovery`` - Recovers automatically from a robot error.

The used messages are:

* ``franka_msgs::action::ErrorRecovery`` - no parameters.

Example usage:::

    ros2 action send_goal /action_server/error_recovery franka_msgs/action/ErrorRecovery {}

Known Issues
------------

* When using the ``fake_hardware`` with MoveIt, it takes some time until the default position is applied.
