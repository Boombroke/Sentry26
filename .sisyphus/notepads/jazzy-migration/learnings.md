# Learnings — jazzy-migration

## [2026-04-15] Session Start
- Plan has 24 implementation tasks + 4 final verification tasks
- Wave 0 and Wave 1 (T1-T4) can be executed on current Humble environment
- Critical blocker: Point-LIO find_package(PythonLibs) — CMake 3.28 (Ubuntu 24.04) removed this
- TEB confirmed unused: both nav2_params.yaml use omni_pid_pursuit_controller only
- fake_vel_transform README is outdated (mentions local_plan subscription) but source code already removed it
- No root-level .clang-format — each sub-package has its own
- rm_serial_driver uses C++14 (verified in CMakeLists.txt)
- All Wave 2 tasks need Jazzy environment installed
- Gazebo migration: rmoss upstream has no Jazzy branches — must port in-house
