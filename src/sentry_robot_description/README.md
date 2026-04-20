# sentry_robot_description

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

SMBU PolarBear Team robot description package for RoboMaster 2026 sentry navigation.

深圳北理莫斯科大学北极熊战队 - RoboMaster 2026 哨兵机器人描述包。

## 1. Overview

本包负责维护 2026 赛季差速轮足哨兵机器人的 TF / joint 结构描述，并在 launch 中把 xmacro 生成的 SDF 转成 URDF 提供给 `robot_state_publisher`。Gazebo 仿真 spawn 也从这里读取机器人描述。

本项目使用 [xmacro](https://github.com/gezp/xmacro) 格式描述机器人关节信息，可以更灵活的组合已有模型。

当前机器人描述文件为 2026 赛季差速轮足底盘定制，不再基于旧 Mecanum 全向模板。模型按环境拆分为共享核心和两个入口文件，避免 Gazebo 物理 workaround 污染实车 TF。

- [wheeled_biped_core](./resource/xmacro/wheeled_biped_core.sdf.xmacro)

    内部共享核心，只定义差速轮足拓扑、`base_footprint → chassis → gimbal_yaw → gimbal_pitch → front_mid360` TF 链、左右轮和云台关节。不要直接作为 launch 入口使用。

- [wheeled_biped_real](./resource/xmacro/wheeled_biped_real.sdf.xmacro)

    实车 TF 入口，也是 `robot_description_launch.py` 和 `sentry_nav_bringup/robot_state_publisher_launch.py` 的默认入口。只保留实车 TF/visual/collision，不包含 Gazebo `sensor`、caster、DiffDrive plugin 或仿真专用下俯角。

- [wheeled_biped_sim](./resource/xmacro/wheeled_biped_sim.sdf.xmacro)

    Gazebo 入口，由 `rmu_gazebo_simulator/spawn_robots.launch.py` 使用。包含仿真专用内容：65% chassis 盒体、前后 caster ball、Gazebo DiffDrive / joint controller plugin、`gpu_lidar` / camera sensor，以及用于覆盖近场低矮底座的 Mid360 固定下俯约 30°。

- [wheeled_biped](./resource/xmacro/wheeled_biped.sdf.xmacro)

    向后兼容别名，当前等价于 `wheeled_biped_real`。新配置应显式使用 `wheeled_biped_real` 或 `wheeled_biped_sim`。

差速底盘（左右两个驱动轮）+ 独立云台 yaw/pitch 关节。Livox Mid360 与 industrial_camera 均挂在 `gimbal_pitch`，随云台旋转。底盘朝向即运动方向（车头始终朝前）。

TF 链（6 层）：`map → odom → base_footprint → chassis → gimbal_yaw → gimbal_pitch → front_mid360`

Mid360 允许相对 `gimbal_pitch` 存在固定安装外参（例如下俯、侧装、倒装）；这些姿态只应写在 `gimbal_pitch → front_mid360` 这一层，不改变底盘或云台的机械语义。实车外参写入 `wheeled_biped_real.sdf.xmacro`，仿真外参写入 `wheeled_biped_sim.sdf.xmacro`。

注意这只是传感器安装外参；Point-LIO 的 `gravity / gravity_init` 不用于表达 Mid360 安装角。仿真保持标准 `[0, 0, -9.81]`，实车由工具箱静态 IMU 标定根据 `mean_acc` 计算重力方向。

仿真放置注意：不要把 Gazebo spawn 的 `-z` 简单理解成“平台面高度”。当前项目以 `gz_world.yaml` 中各世界实测稳定的 `z_pose` 为准，其中 `rmuc_2026` 的轮足模型已验证需要 `0.72`，否则轮子会卡进地图。

## 2. Which File To Edit

日常开发时，不要再直接改 legacy 的 `wheeled_biped.sdf.xmacro`。按场景改下面这三个文件：

| 需求 | 应修改文件 | 典型修改项 |
|---|---|---|
| 改实车雷达安装角、倒装/斜装、实测平移外参 | [wheeled_biped_real.sdf.xmacro](./resource/xmacro/wheeled_biped_real.sdf.xmacro) | `front_lidar_pose` |
| 改 Gazebo 专用 workaround | [wheeled_biped_sim.sdf.xmacro](./resource/xmacro/wheeled_biped_sim.sdf.xmacro) | 仿真 LiDAR 下俯角、仿真底盘尺寸 |
| 改机器人公共结构 | [wheeled_biped_core.sdf.xmacro](./resource/xmacro/wheeled_biped_core.sdf.xmacro) | 轮距、轮径、云台高度、公共 link/joint/TF 拓扑 |

### 2.1 常见判断

- LiDAR 在实车上斜装、倒装了：改 `wheeled_biped_real.sdf.xmacro`
- 仿真里低矮底座扫不到，想保留 30° 下俯或继续调它：改 `wheeled_biped_sim.sdf.xmacro`
- 轮半径、轮距、云台高度这些实车/仿真都应该一致的量变了：改 `wheeled_biped_core.sdf.xmacro`
- Point-LIO 里的 `gravity` 不对：去改导航参数或用工具箱重力标定，不要来改 xmacro

### 2.2 现在默认谁在用哪个入口

- `ros2 launch sentry_robot_description robot_description_launch.py`
  - 默认 `robot_name:=wheeled_biped_real`
- `ros2 launch sentry_nav_bringup robot_state_publisher_launch.py`
  - 默认 `robot_name:=wheeled_biped_real`
- `ros2 launch rmu_gazebo_simulator bringup_sim.launch.py`
  - 内部固定使用 `wheeled_biped_sim.sdf.xmacro`

### 2.3 为什么不建议改包名

`sentry_robot_description` 这个名字在 ROS 生态里是标准、直观且低歧义的命名方式，语义就是“这个包提供机器人描述”。从职责上看，它现在仍然是恰当的。

当前**不建议**把它重命名，原因是：

- 它已经被多个 launch 和 package 直接依赖，比如 `get_package_share_directory("sentry_robot_description")`
- Gazebo spawn、Nav2 bringup、依赖声明、文档索引都已经稳定使用这个名字
- 改名收益很小，只是文字更贴近“哨兵轮足”，但风险是真实的：会带来 launch、依赖、文档和安装路径的同步修改成本

如果后面真的要改名，更合适的目标也不是现在立刻做，而是等描述结构和赛季模型稳定后，再统一重命名为更赛季化的名字，例如 `sentry_wheeled_biped_description`。现阶段保留 `sentry_robot_description` 是更稳妥的选择。

## 3. Quick Start

### 3.1 Setup Environment

- Ubuntu 24.04
- ROS: [Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### 3.2 Create Workspace

```bash
sudo apt install git-lfs
pip install vcstool2
```

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/sentry_robot_description.git
```

```bash
vcs import --recursive < dependencies.repos
```

```bash
pip install xmacro
```

> Ubuntu 24.04 上通常需要：
>
> ```bash
> pip3 install xmacro --break-system-packages
> ```

### 3.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install
```

### 3.4 Running

#### Option 1: 在 RViz 中可视化实车 TF 模型

```bash
ros2 launch sentry_robot_description robot_description_launch.py
```

如果想直接查看仿真描述入口：

```bash
ros2 launch sentry_robot_description robot_description_launch.py robot_name:=wheeled_biped_sim
```

#### Option 2: Python API

通过 Python API，在 launch file 中解析 XMacro 文件，生成 URDF 和 SDF 文件 (Recommend)：

> [!TIP]
>
> [robot_state_publisher](https://github.com/ros/robot_state_publisher) 需要传入 urdf 格式的机器人描述文件
>
> Gazebo 仿真器 spawn robot 时，需要传入 sdf / urdf 格式的机器人描述文件

感谢前辈的开源工具 [xmacro](https://github.com/gezp/xmacro) 和 [sdformat_tools](https://github.com/gezp/sdformat_tools) ，这里简述 XMacro 转 URDF 和 SDF 的示例，用于在 launch file 中生成 URDF 和 SDF 文件。

```python
from xmacro.xmacro4sdf import XMLMacro4sdf
from sdformat_tools.urdf_generator import UrdfGenerator

xmacro = XMLMacro4sdf()
xmacro.set_xml_file(robot_xmacro_path)

# Generate SDF from xmacro
xmacro.generate()
robot_xml = xmacro.to_string()

# Generate URDF from SDF
urdf_generator = UrdfGenerator()
urdf_generator.parse_from_sdf_string(robot_xml)
robot_urdf_xml = urdf_generator.to_string()
```

#### Option 3: 命令行

通过命令行直接转换输出 SDF 文件（Not Recommend）:

```bash
source install/setup.bash

xmacro4sdf src/sentry_robot_description/resource/xmacro/wheeled_biped_real.sdf.xmacro > /tmp/wheeled_biped_real.sdf
xmacro4sdf src/sentry_robot_description/resource/xmacro/wheeled_biped_sim.sdf.xmacro > /tmp/wheeled_biped_sim.sdf
```

## 4. Subscribed Topics

None.

## 5. Published Topics

- `robot_description (std_msgs/msg/String)`

    机器人描述文件（字符串形式）。

- `joint_states (sensor_msgs/msg/JointState)`

    如果命令行中未给出 URDF，则此节点将侦听 `robot_description` 话题以获取要发布的 URDF。一旦收到一次，该节点将开始将关节状态发布到 `joint_states` 话题。

- `any_topic (sensor_msgs/msg/JointState)`

    如果 `sources_list` 参数不为空（请参阅下面的参数），则将订阅此参数中的每个命名话题以进行联合状态更新。不要将默认的 `joint_states` 话题添加到此列表中，因为它最终会陷入无限循环。

- `tf, tf_static (tf2_msgs/msg/TFMessage)`

    机器人关节坐标系信息。

## 6. Launch Arguments

- `use_sim_time` (bool, default: False)

    是否使用仿真时间。

- `robot_name` (str, default: "wheeled_biped_real")

    机器人 XMacro 描述文件的**名字（无需后缀）**。描述文件应位于 `package://sentry_robot_description/resource/xmacro` 目录下。

- `robot_xmacro_file` (str, default: "[wheeled_biped_real.sdf.xmacro](./resource/xmacro/wheeled_biped_real.sdf.xmacro)")

    机器人 XMacro 描述文件的**绝对路径**。本参数的优先级高于 `robot_name`，即若设置了 `robot_xmacro_file`，则 `robot_name` 参数无效。若未设置 `robot_xmacro_file`，则使用 `robot_name` 参数并自动补全路径作为 `robot_xmacro_file` 的值。

- `params_file` (str, default: [robot_description.yaml](./params/robot_description.yaml))

- `rviz_config_file` (str, default: [visualize_robot.rviz](./rviz/visualize_robot.rviz))

    RViz 配置文件路径。

- `use_rviz` (bool, default: True)

    是否启动 RViz 可视化界面。

- `use_respawn` (bool, default: False)

    是否在节点退出时尝试重启节点。

- `log_level` (str, default: "info")

    日志级别。
