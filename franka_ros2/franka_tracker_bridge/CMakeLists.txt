cmake_minimum_required(VERSION 3.10)

project(franka_tracker_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

get_filename_component(
  DEFAULT_LIBSURVIVE_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../libsurvive"
  ABSOLUTE
)
set(LIBSURVIVE_ROOT "${DEFAULT_LIBSURVIVE_ROOT}" CACHE PATH "libsurvive checkout root")
find_path(LIBSURVIVE_INCLUDE_DIR
  NAMES libsurvive/survive_api.h
  HINTS "${LIBSURVIVE_ROOT}/include"
  REQUIRED
)
find_library(LIBSURVIVE_LIBRARY
  NAMES survive
  HINTS "${LIBSURVIVE_ROOT}/build"
  REQUIRED
)
set(LIBSURVIVE_REDIST_DIR "${LIBSURVIVE_ROOT}/redist")
set(LIBSURVIVE_CNMATRIX_INCLUDE_DIR "${LIBSURVIVE_ROOT}/libs/cnmatrix/include")
set(LIBSURVIVE_CNKALMAN_INCLUDE_DIR "${LIBSURVIVE_ROOT}/libs/cnkalman/include")

add_executable(libsurvive_pose_node src/libsurvive_pose_node.cpp)
target_include_directories(libsurvive_pose_node PRIVATE
  include
  "${LIBSURVIVE_INCLUDE_DIR}"
  "${LIBSURVIVE_REDIST_DIR}"
  "${LIBSURVIVE_CNMATRIX_INCLUDE_DIR}"
  "${LIBSURVIVE_CNKALMAN_INCLUDE_DIR}"
)
target_link_libraries(libsurvive_pose_node "${LIBSURVIVE_LIBRARY}")
ament_target_dependencies(libsurvive_pose_node rclcpp geometry_msgs std_msgs)
target_compile_definitions(libsurvive_pose_node PRIVATE
  FRANKA_TRACKER_BRIDGE_DEFAULT_LIBSURVIVE_ROOT="${LIBSURVIVE_ROOT}"
)
set_target_properties(libsurvive_pose_node PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  BUILD_RPATH "${LIBSURVIVE_ROOT}/build"
  INSTALL_RPATH "${LIBSURVIVE_ROOT}/build"
)

install(TARGETS
  libsurvive_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/tracker_meshcat_preview_node
  scripts/tracker_pose_target_node
  scripts/tracker_debug_curve_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_tracker_meshcat_preview_core test/test_tracker_meshcat_preview_core.py)
  ament_add_pytest_test(test_tracker_pose_target_core test/test_tracker_pose_target_core.py)
  ament_add_pytest_test(test_relative_target_contract test/test_relative_target_contract.py)
  ament_add_pytest_test(test_tracker_preview_config test/test_tracker_preview_config.py)
  ament_add_pytest_test(test_debug_curve_core test/test_debug_curve_core.py)
  ament_add_pytest_test(test_debug_curve_node_static test/test_debug_curve_node_static.py)

  ament_add_gtest(test_libsurvive_pose_conversions test/test_libsurvive_pose_conversions.cpp)
  target_include_directories(test_libsurvive_pose_conversions PRIVATE include)
  ament_target_dependencies(test_libsurvive_pose_conversions rclcpp geometry_msgs std_msgs)
endif()

ament_package()
