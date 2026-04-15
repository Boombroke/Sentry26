# sentry_behavior

## 简介
sentry_behavior 是哨兵机器人的行为决策系统。本系统基于 BehaviorTree.CPP 4.9.0 和 BehaviorTree.ROS2 0.3.0 框架开发。它包含服务端与客户端逻辑，服务端继承自 TreeExecutionServer 类，通过 action server 接收并执行行为树请求。

## 主要功能
系统通过订阅裁判系统话题实时获取比赛状态，并将关键数据写入全局黑板供行为树节点调用。它集成了多种自定义行为树插件，支持复杂的战术逻辑编排与状态切换。

## 行为树插件分类
- **Conditions (条件节点)**:
  - is_game_status: 判断当前比赛阶段。
  - is_rfid_detected: 检测是否处于 RFID 增益点。
  - is_status_ok: 检查机器人硬件与软件状态。
  - is_attacked: 判断机器人是否受到敌方攻击。
  - is_detect_enemy: 视觉系统是否发现敌方目标。
  - is_hp_add: 判断机器人是否正在执行补血操作。
  - is_outpost_ok: 检查己方前哨站的存活状态。
- **Actions (动作节点)**:
  - pub_goal: 向导航堆栈发布目标点坐标。
  - battlefield_information: 处理并更新战场全局信息。
  - pursuit: 执行对敌方目标的追踪动作。
- **Controls (控制节点)**:
  - recovery_node: 任务失败时的故障恢复逻辑。
- **Decorators (装饰节点)**:
  - rate_controller: 控制子树的运行频率。
  - tick_after_timeout_node: 在指定超时时间后触发 tick。

## 订阅话题
- referee/game_status: 接收比赛阶段与剩余时间。
- referee/robot_status: 获取机器人等级、血量等信息。
- referee/rfid_status: 监控增益点占领情况。

## 发布话题
- behavior_tree_log: 发布行为树节点状态切换日志。

## 参数说明
- action_name: 行为树 action 服务的注册名称。
- tick_frequency: 行为树主循环的运行频率。
- groot2_port: 用于 Groot2 远程可视化调试的端口。
- use_cout_logger: 是否在终端实时打印节点状态切换。
- plugins: 运行时需要动态加载的 C++ 插件库列表。
- behavior_trees: 行为树 XML 描述文件的存储路径。

## 使用方法
执行以下命令启动决策节点：
```bash
ros2 launch sentry_behavior sentry_behavior_launch.py
```

## 目录结构概述
- behavior_trees/: 存放所有战术逻辑的 XML 定义文件。
- include/: 包含核心类定义与插件接口头文件。
- plugins/: 存放各类行为树插件的源代码实现。
- src/: 包含 TreeExecutionServer 的具体实现逻辑。
