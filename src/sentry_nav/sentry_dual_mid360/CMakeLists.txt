cmake_minimum_required(VERSION 3.10)
project(sentry_dual_mid360)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

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

# Install executable
install(TARGETS merger_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY urdf config launch scripts docs
  DESTINATION share/${PROJECT_NAME}
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

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
