# 远程调试指南

通过 Foxglove Studio + foxglove_bridge 实现远程 Web 可视化，无需在远程机器上安装 ROS2，浏览器即可查看 costmap、路径、TF、点云等所有导航信息。

## 1. 服务端（机器人 / 运行导航的电脑）

### 1.1 安装 foxglove_bridge

```bash
sudo apt install ros-humble-foxglove-bridge
```

### 1.2 启动导航时启用 Foxglove

仿真环境：
```bash
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py use_foxglove:=true
```

实车环境：
```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py use_foxglove:=true
```

可以同时启用 RViz 和 Foxglove：
```bash
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py use_rviz:=true use_foxglove:=true
```

无头模式（无显示器的实车推荐）：
```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py use_rviz:=false use_foxglove:=true
```

### 1.3 单独启动（不随 launch 文件）

如果导航已经在运行，可以单独启动 bridge：
```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args \
    -p port:=8765 \
    -p address:="0.0.0.0"
```

### 1.4 确认服务端正常

启动后应看到类似日志：
```
[foxglove_bridge]: Starting Foxglove bridge on ws://0.0.0.0:8765
```

确认端口监听：
```bash
ss -tlnp | grep 8765
```

### 1.5 防火墙放行（如有防火墙）

```bash
sudo ufw allow 8765/tcp
```

---

## 2. 客户端（远程调试的电脑 / 手机 / 平板）

### 2.1 方式一：Web 版（推荐，零安装）

1. 打开浏览器（Chrome / Edge / Firefox）
2. 访问 **https://app.foxglove.dev**
3. 点击 **"Open connection"**
4. 选择 **"Foxglove WebSocket"**
5. 输入地址：**`ws://<机器人IP>:8765`**

   例如机器人 IP 是 `192.168.1.100`：
   ```
   ws://192.168.1.100:8765
   ```

6. 点击 **"Open"**

> **注意**：Web 版需要 HTTPS 页面连接 WSS 才安全。内网调试用 HTTP 即可，Chrome 可能会提示不安全连接，点击"仍然继续"。如果遇到 mixed content 限制，使用桌面版。

### 2.2 方式二：桌面版

1. 下载 Foxglove Studio：https://foxglove.dev/download
2. 安装并打开
3. 点击 **"Open connection"** → **"Foxglove WebSocket"**
4. 输入 `ws://<机器人IP>:8765`
5. 点击 **"Open"**

桌面版没有浏览器的安全限制，连接更稳定。

### 2.3 方式三：手机 / 平板

直接用手机浏览器访问 https://app.foxglove.dev，步骤同 Web 版。适合比赛时在场边快速查看。

---

## 3. 查看导航信息

连接成功后，添加以下面板查看对应数据：

### 3.1 推荐面板布局

| 面板类型 | 用途 | 订阅话题 |
|---------|------|---------|
| **3D** | 地图 + 路径 + 机器人 + 点云 | `/map`, `/plan`, `/tf`, `/scan` |
| **Map** | 2D costmap 细节 | `/global_costmap/costmap` 或 `/local_costmap/costmap` |
| **Plot** | 速度/位置曲线 | `/cmd_vel`, `/odometry` |
| **Log** | ROS 日志 | `/rosout` |
| **Topic List** | 话题频率监控 | 自动发现 |

### 3.2 添加 3D 面板步骤

1. 点击左上角 **"+"** 添加面板
2. 选择 **"3D"**
3. 在右侧面板设置中启用：
   - **Grid** → 显示地面网格
   - **Map** → 选择 `/map` 话题（静态地图）
   - **Occupancy Grid** → 选择 `/global_costmap/costmap`（全局 costmap）
   - **Path** → 选择 `/plan`（全局路径）
   - **LaserScan** → 选择 `/scan`（激光扫描）
   - **TF** → 自动显示坐标系

### 3.3 发送导航目标点

需要从远程发送目标点：

1. 添加 **"Publish"** 面板
2. 话题设为 `/goal_pose`
3. 消息类型 `geometry_msgs/msg/PoseStamped`
4. 填入目标坐标后点击 Publish

或使用 3D 面板中的 **"Publish Point"** 工具直接在地图上点击发送。

---

## 4. 网络连接场景

### 4.1 同一局域网（最常见）

机器人和调试电脑连同一个 WiFi / 路由器：

```
调试电脑 (192.168.1.50)  ←→  WiFi  ←→  机器人 (192.168.1.100)
                                          foxglove_bridge :8765
```

客户端连接：`ws://192.168.1.100:8765`

### 4.2 USB 直连（实车调试）

用网线或 USB 直连机器人 NUC：

```bash
# 机器人端查看 IP
ip addr show
# 假设是 10.42.0.1
```

客户端连接：`ws://10.42.0.1:8765`

### 4.3 跨网段 / 外网（通过 SSH 隧道）

机器人不在同一局域网时，用 SSH 端口转发：

```bash
# 在调试电脑上执行
ssh -L 8765:localhost:8765 <user>@<机器人公网IP或跳板机>
```

客户端连接：`ws://localhost:8765`

### 4.4 跨网段 / 外网（通过 Tailscale VPN）

```bash
# 机器人和调试电脑都安装 Tailscale
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up

# 查看 Tailscale 分配的 IP
tailscale ip -4
# 假设机器人是 100.64.0.2
```

客户端连接：`ws://100.64.0.2:8765`

---

## 5. 性能优化

### 5.1 带宽瓶颈排查

如果远程可视化卡顿，主要消耗带宽的话题是：
- `PointCloud2`（~5MB/s @ 10Hz）
- `OccupancyGrid / costmap`（~0.5MB/s @ 5Hz）
- `Image`（~10MB/s 未压缩）

### 5.2 在 Foxglove 中降频

在 Foxglove Studio 面板设置中，可以对每个话题设置 **订阅频率限制**（Topic Settings → Message Rate），例如将点云从 10Hz 降到 2Hz。

### 5.3 服务端限制话题

如果带宽非常有限（如 4G 网络），启动时可手动指定只暴露必要话题：

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args \
    -p port:=8765 \
    -p address:="0.0.0.0" \
    -p topic_whitelist:="['/map', '/plan', '/tf', '/tf_static', '/scan', '/global_costmap/costmap', '/local_costmap/costmap', '/odometry', '/cmd_vel', '/rosout']"
```

---

## 6. 故障排查

| 现象 | 原因 | 解决 |
|------|------|------|
| 连接失败 | 防火墙阻止 8765 端口 | `sudo ufw allow 8765/tcp` |
| 连接失败 | IP 地址错误 | 在机器人端 `ip addr show` 确认 IP |
| 浏览器 Web 版报 mixed content | HTTPS 页面不允许 ws:// | 改用桌面版，或用 wss:// + TLS 证书 |
| 看不到话题 | namespace 不匹配 | 话题带 namespace 前缀，如 `/red_standard_robot1/map` |
| 3D 面板空白 | 没有选择 Fixed Frame | 在 3D 面板设置中把 Frame 改为 `map` |
| 点云/costmap 卡顿 | 带宽不够 | 降频或用 topic_whitelist 限制 |
| TF 报错 | use_sim_time 不匹配 | 确认 foxglove_bridge 和导航栈的 use_sim_time 一致 |

---

## 7. 与 RViz 的功能对比

| 功能 | RViz2 | Foxglove Web |
|------|-------|-------------|
| costmap 显示 | ✅ | ✅ |
| 路径显示 | ✅ | ✅ |
| TF 树 | ✅ | ✅ |
| 点云 | ✅ | ✅ |
| 激光扫描 | ✅ | ✅ |
| URDF 模型 | ✅ | ✅ |
| 2D Nav Goal | ✅ 鼠标点击 | ✅ Publish 面板 |
| 参数动态调整 | ❌ 需 rqt | ✅ Parameters 面板 |
| 话题频率监控 | ❌ 需 ros2 topic hz | ✅ Topic List 面板 |
| 远程免安装 | ❌ 需 ROS2 环境 | ✅ 浏览器即可 |
| 录制回放 | ❌ 需 ros2 bag | ✅ 内置录制 |
