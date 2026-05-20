cmake_minimum_required(VERSION 3.10)
project(small_gicp_relocalization)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

## NOTE: do not switch back to `DIRECTORY src` — `build_scan_context_db.cpp`
## defines main() and must not be linked into the shared library.
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/scan_context.cpp
  src/small_gicp_relocalization.cpp
)

target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_gicp_relocalization::SmallGicpRelocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

## Offline tool: build a Scan Context descriptor database from a prior PCD map.
ament_auto_add_executable(build_scan_context_db
  src/build_scan_context_db.cpp
)
target_link_libraries(build_scan_context_db ${PROJECT_NAME})

#############
## Testing ##
#############

if(BUILD_TESTING)
set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
