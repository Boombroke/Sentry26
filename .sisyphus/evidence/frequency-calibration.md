# T24: 频率校准验证

## 当前配置

| 参数 | 仿真 | 实车 |
|------|------|------|
| controller_frequency | 20.0 Hz | 30.0 Hz |
| smoothing_frequency | 20.0 Hz | 30.0 Hz |
| 匹配 | ✅ | ✅ |

## 分析
- 仿真/实车配置均满足 AGENTS.md §9 的频率匹配要求
- 实车 50Hz 曾导致严重卡顿（实测仅达 27Hz，最大间隔 22s），30Hz 为验证安全值
- Jazzy 下执行器行为可能与 Humble 不同，需运行时验证实际频率达成率

## 建议
不修改参数。部署后用 `ros2 topic hz /cmd_vel --window 10` 验证实际频率。
