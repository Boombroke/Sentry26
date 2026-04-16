# rm_interfaces

项目统一 ROS2 自定义消息包，包含裁判系统和视觉系统接口。

## 消息列表

### 裁判系统消息 (msg/referee/)

对应串口协议版本：[V1.7.0 (20241225)](https://terra-1-g.djicdn.com/b2a076471c6c4b72b574a977334d3e05/RoboMaster%20%E8%A3%81%E5%88%A4%E7%B3%BB%E7%BB%9F%E4%B8%B2%E5%8F%A3%E5%8D%8F%E8%AE%AE%E9%99%84%E5%BD%95%20V1.7.0%EF%BC%8820241225%EF%BC%89.pdf)

| 消息 | 说明 | 发布者 |
|------|------|--------|
| GameStatus.msg | 比赛阶段与剩余时间 | rm_serial_driver |
| RobotStatus.msg | 机器人血量、等级、伤害、弹量 | rm_serial_driver |
| RfidStatus.msg | RFID 增益点检测状态 | rm_serial_driver |
| GameRobotHP.msg | 己方 7 个单位血量 | rm_serial_driver |

### 视觉消息 (msg/vision/)

| 消息 | 说明 | 消费者 |
|------|------|--------|
| Armor.msg | 单个装甲板几何与分类信息 | — (Armors 内嵌类型) |
| Armors.msg | 装甲板检测结果数组 | sentry_behavior (is_detect_enemy) |
| Target.msg | 跟踪目标运动状态与预测位置 | sentry_behavior (pursuit) |

## 编译

```bash
colcon build --packages-select rm_interfaces --symlink-install
```

## 依赖

- `geometry_msgs`
- `std_msgs`
