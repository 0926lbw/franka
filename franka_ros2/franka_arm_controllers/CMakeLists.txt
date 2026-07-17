cmake_minimum_required(VERSION 3.10)
project(franka_arm_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(urdf REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/joint_impedance_pose_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  franka_msgs
  franka_semantic_components
  geometry_msgs
  hardware_interface
  kdl_parser
  moveit_msgs
  orocos_kdl
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tinyxml2_vendor
  TinyXML2
  urdf
)

pluginlib_export_plugin_description_file(controller_interface franka_arm_controllers.xml)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/move_to_joint_goal.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_to_joint_goal
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  controller_interface
  franka_msgs
  franka_semantic_components
  geometry_msgs
  hardware_interface
  kdl_parser
  moveit_msgs
  orocos_kdl
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tinyxml2_vendor
  urdf
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(kdl_joint_state_wiring test/test_kdl_joint_state_wiring.py)
  ament_add_pytest_test(slow_joint_move test/test_slow_joint_move.py)
endif()

ament_package()
