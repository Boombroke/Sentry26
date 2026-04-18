# 日志归档目录

本目录同时保存 `archive_log.sh` 归档的 ROS2 launch 日志，以及 `record_nav_bag.sh` 录制的 Nav Goal rosbag。
**已加入 `.gitignore`，不随仓库提交**。

## 日志来源层级

| 层级 | 位置 | 内容 |
|---|---|---|
| **本目录** `logs/` | `logs/<timestamp>-<label>/` | 你手动归档的 launch 运行日志（本工具维护） |
| ROS2 默认 | `~/.ros/log/<timestamp>-<hostname>-<pid>/` | 每次 `ros2 launch` 自动生成，每节点一个文件 |
| ROS2 散落 | `~/.ros/log/*.log` | 单独运行的 ros2 node / ros2 topic 等命令的日志 |
| Colcon 编译 | `<project_root>/log/latest_build/` | `colcon build` 的编译日志 |
| Gazebo 内部 | `~/.gz/sim/log/<timestamp>/` | gz-sim 自己的服务器日志 |

## 常用 debug 命令

所有辅助脚本在 `src/scripts/debug/`：

### 实时跟随最新 launch 日志

```bash
# 跟随 launch.log（所有节点的 stdout 合流，最方便）
bash src/scripts/debug/tail_log.sh

# 跟随指定节点（按名字模糊匹配）
bash src/scripts/debug/tail_log.sh controller_server
bash src/scripts/debug/tail_log.sh pointlio
bash src/scripts/debug/tail_log.sh odom_bridge
```

### 在最新 launch 日志里搜关键字

```bash
bash src/scripts/debug/grep_log.sh error
bash src/scripts/debug/grep_log.sh "TF lookup"
bash src/scripts/debug/grep_log.sh "imu loop"
bash src/scripts/debug/grep_log.sh SUCCEEDED
```

### 归档当前 launch 的日志到项目 logs/

```bash
# 默认标签 "run"
bash src/scripts/debug/archive_log.sh

# 自定义标签（便于查找）
bash src/scripts/debug/archive_log.sh nav_goal_test
bash src/scripts/debug/archive_log.sh slam_build
```

### 录制 / 分析 / 清理 Nav Goal rosbag

```bash
bash src/scripts/debug/record_nav_bag.sh [label] [duration] [goal_x] [goal_y]
python3 src/scripts/debug/analyze_bag.py [bag_dir]     # 省略则取 logs/ 最新
bash src/scripts/debug/clean_logs.sh [--keep N|--all]  # 省略则列出
```

## 典型 Debug 流程

### 情景 1：Nav Goal 被拒 / 不动

```bash
# 1. 先看 launch.log 是否有 ERROR
bash src/scripts/debug/grep_log.sh ERROR

# 2. 看控制器和规划器
bash src/scripts/debug/grep_log.sh "controller_server"
bash src/scripts/debug/grep_log.sh "planner_server"

# 3. 看 TF 链是否完整
bash src/scripts/debug/grep_log.sh "base_footprint"
bash src/scripts/debug/grep_log.sh "Timed out waiting for transform"

# 4. 归档当次运行以便后续对比
bash src/scripts/debug/archive_log.sh failed_goal
```

### 情景 2：Point-LIO 起不来

```bash
bash src/scripts/debug/grep_log.sh "IMU Initializing"
bash src/scripts/debug/grep_log.sh "imu loop back"
bash src/scripts/debug/tail_log.sh pointlio
```

### 情景 3：对比两次运行

```bash
# 第一次
bash src/scripts/debug/archive_log.sh baseline

# 改参数，重启仿真...

# 第二次
bash src/scripts/debug/archive_log.sh tuned

# 对比
diff logs/*-baseline/launch.log logs/*-tuned/launch.log | head -60
```


## 日志文件说明

每次 `ros2 launch` 生成的目录（`~/.ros/log/<ts>/`）结构：

```
2026-04-18-15-57-16-760293-pc-XiaoXinPro-14-IMH9-534754/
├── launch.log                         # 所有节点输出合流（最常用）
├── launch-build-info.log             # 编译信息
├── <node_name>-<N>-stdout.log        # 某节点的 stdout
├── <node_name>-<N>-stderr.log        # 某节点的 stderr
└── <node_name>-<N>-stdout_stderr.log # 合并流
```

示例节点名：
- `component_container_isolated-19-stdout.log` → Nav2 合并容器
- `pointlio_mapping-24-stdout.log` → Point-LIO
- `sync_slam_toolbox_node-23-stdout.log` → slam_toolbox
- `gazebo-1-stdout.log` → Gazebo
- `rviz2-28-stdout.log` → RViz

## 清理旧日志

`~/.ros/log/` 会无限积累，建议定期清理：

```bash
# 保留最近 7 天
find ~/.ros/log -maxdepth 1 -type d -mtime +7 -exec rm -rf {} +

# 保留最近 10 次
cd ~/.ros/log && ls -td */ | tail -n +11 | xargs -r rm -rf
```
