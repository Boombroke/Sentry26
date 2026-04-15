# T22: terrain_analysis/ext 不可组合 — 跳过

## 结论
terrain_analysis 和 terrain_analysis_ext 是独立可执行文件（standalone executable），不是 composable node，无法加载到 nav2_container。

## 证据
- CMakeLists.txt 使用 `add_executable()`，不是 `add_library()` + `rclcpp_components_register_node()`
- 源码使用传统 `int main()` + `while(rclcpp::ok()) { rate.sleep(); }` 循环
- 使用大量文件作用域全局变量（terrainAnalysis.cpp 约 700 行）
- navigation_launch.py 中以 Node() 启动，在 composition 块之外

## 重构代价
将其转为 composable node 需要：封装全部全局变量为类成员、替换 spin loop 为 timer callback、添加 rclcpp_components 依赖。这是对上游 CMU 代码的重大重构，不在本次迁移范围内。
