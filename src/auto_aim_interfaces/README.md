# auto_aim_interfaces

### 简介
本包定义了自动瞄准系统专用的 ROS2 消息接口。它为视觉识别与目标跟踪模块提供了统一的数据交换格式。

### 功能
包内包含多种自定义消息，涵盖了装甲板识别、目标状态及调试信息。

### 消息类型
*   Armor.msg: 描述单个装甲板的几何与分类信息。
*   Armors.msg: 包含多个装甲板的数组。
*   Target.msg: 记录跟踪目标的运动状态与预测位置。
*   TrackerInfo.msg: 提供跟踪器的内部调试参数。
*   DebugArmor.msg / DebugArmors.msg: 用于可视化调试的装甲板数据。
*   DebugLight.msg / DebugLights.msg: 用于可视化调试的灯条数据。

### 使用方法
这是一个纯接口包，不包含任何可执行节点。编译时请运行以下命令：
```bash
colcon build --packages-select auto_aim_interfaces
```
其他功能包在 package.xml 中添加依赖后即可引用这些消息。
