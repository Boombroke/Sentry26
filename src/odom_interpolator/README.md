# odom_interpolator

### 简介
本包实现了里程计数据的插值处理与差速分析功能。它能提升里程计数据的更新频率，并提供速度偏差的监控手段。

### 功能
*   将低频里程计输入转换为高频插值输出。
*   实时计算里程计反馈速度与指令速度之间的差值。
*   发布可视化 marker，在 RViz 中以箭头形式展示速度矢量。

### 话题
*   订阅:
    *   odom_topic: 原始里程计数据。
    *   cmd_vel_topic: 机器人速度控制指令。
*   发布:
    *   interpolated_odom_topic: 插值后的高频里程计。
    *   diff_vel_topic: 速度偏差数据。
    *   velocity_marker_topic: 用于显示的 marker 数据。

### 参数
*   interpolation_rate: 插值输出频率，默认值为 100Hz。
*   arrow_scale: RViz 中速度箭头的显示缩放比例。

### 使用方法
使用 launch 文件启动节点：
```bash
ros2 launch odom_interpolator odom_interpolator_launch.py
```
