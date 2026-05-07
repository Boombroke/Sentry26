cmake_minimum_required(VERSION 3.10)
project(sentry_dual_mid360)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
ament_auto_find_build_dependencies()

set(POINTLIO_OVERRIDE_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/config")
set(POINTLIO_OVERRIDE_OUTPUT "${POINTLIO_OVERRIDE_OUTPUT_DIR}/pointlio_dual_overrides.yaml")
set(POINTLIO_OVERRIDE_GENERATOR "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_pointlio_overrides.py")
set(POINTLIO_OVERRIDE_IMU_XMACRO "${CMAKE_CURRENT_SOURCE_DIR}/urdf/mid360_imu_tf.sdf.xmacro")

# Keep sentry_dual_mid360 independently buildable: locate the real robot xmacro
# through the repository src/ layout instead of requiring sentry_robot_description
# to be installed before this package is built.
get_filename_component(SENTRY26_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(POINTLIO_OVERRIDE_REAL_XMACRO
  "${SENTRY26_SRC_DIR}/sentry_robot_description/resource/xmacro/wheeled_biped_real.sdf.xmacro"
)

add_custom_command(
  OUTPUT ${POINTLIO_OVERRIDE_OUTPUT}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${POINTLIO_OVERRIDE_OUTPUT_DIR}
  COMMAND ${Python3_EXECUTABLE} ${POINTLIO_OVERRIDE_GENERATOR}
          --real-xmacro ${POINTLIO_OVERRIDE_REAL_XMACRO}
          --imu-macro ${POINTLIO_OVERRIDE_IMU_XMACRO}
          --output ${POINTLIO_OVERRIDE_OUTPUT}
  DEPENDS
    ${POINTLIO_OVERRIDE_GENERATOR}
    ${POINTLIO_OVERRIDE_REAL_XMACRO}
    ${POINTLIO_OVERRIDE_IMU_XMACRO}
  COMMENT "Generating Point-LIO dual Mid360 overrides from xmacro inputs"
  VERBATIM
)

add_custom_target(generate_pointlio_dual_overrides ALL
  DEPENDS ${POINTLIO_OVERRIDE_OUTPUT}
)

# Add executable
add_executable(merger_node src/merger_node.cpp)
target_include_directories(merger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(merger_node
  rclcpp
  rclcpp_components
  sensor_msgs
  livox_ros_driver2
  tf2_ros
  tf2_eigen
  message_filters
  std_msgs
)

add_executable(sim_pointcloud_to_custommsg_node src/sim_pointcloud_to_custommsg_node.cpp)
target_include_directories(sim_pointcloud_to_custommsg_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sim_pointcloud_to_custommsg_node
  rclcpp
  rclcpp_components
  sensor_msgs
  livox_ros_driver2
  std_msgs
)

# Install executable
install(TARGETS merger_node sim_pointcloud_to_custommsg_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY urdf config launch scripts docs
  DESTINATION share/${PROJECT_NAME}
)

install(FILES ${POINTLIO_OVERRIDE_OUTPUT}
  DESTINATION share/${PROJECT_NAME}/config
)

# Install scripts defensively as programs (exclude .gitkeep)
file(GLOB SCRIPT_PROGRAMS "scripts/*.py" "scripts/*.sh")
if(SCRIPT_PROGRAMS)
  install(PROGRAMS ${SCRIPT_PROGRAMS}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

# Expose share/sentry_dual_mid360 to IGN_GAZEBO_RESOURCE_PATH so that
# model://urdf/back_mid360.sdf.xmacro and model://urdf/mid360_imu_tf.sdf.xmacro
# can be resolved by xmacro4sdf when rendering wheeled_biped_real.sdf.xmacro.
# This follows the same pattern as sentry_robot_description/env-hooks/gazebo.dsv.in.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/xmacro_path.dsv.in")

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
