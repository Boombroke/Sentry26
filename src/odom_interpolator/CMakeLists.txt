cmake_minimum_required(VERSION 3.5)
project(odom_interpolator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 移除include_directories(include) 或者只保留Eigen3
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(odom_interpolator src/odom_interpolator_node.cpp)

ament_target_dependencies(odom_interpolator
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  Eigen3
)

# Install targets
install(TARGETS odom_interpolator
  DESTINATION lib/${PROJECT_NAME}
)

# 注释掉或移除安装include目录的指令（如果没有头文件）
# install(DIRECTORY include/
#   DESTINATION include/
# )

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()