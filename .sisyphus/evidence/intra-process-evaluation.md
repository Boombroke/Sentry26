# T23: Intra-process 通信评估 — 跳过（风险过高）

## 当前配置
- 使用 `component_container_isolated`（每个节点独立线程）

## 线程安全分析
- **fake_vel_transform**: 有 `angle_mutex_` 保护 `current_robot_base_angle_`，但 `spin_speed_` 无锁（单线程执行器下安全）
- **odom_bridge**: 无互斥锁，依赖 message_filters 同步回调（单线程安全）

## 风险评估
1. `component_container` 用单线程执行器串行化所有节点回调，长耗时回调（如路径规划）会阻塞速度链路
2. message_filters 不支持 intra-process transport
3. Nav2 lifecycle 节点的 publisher wrapper 可能不兼容 intra-process
4. Twist/TwistStamped 消息体积极小（<100字节），zero-copy 收益可忽略

## 结论
保持 `component_container_isolated`。回调串行化风险远大于 zero-copy 收益。
