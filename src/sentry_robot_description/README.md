# sentry_robot_description

2026 赛季哨兵差速轮足机器人的描述包。

这个包的职责只有两件事：

1. 定义机器人公共 TF / joint / link 拓扑  
2. 提供实车与 Gazebo 两套描述入口，分别给 `robot_state_publisher` 和仿真 spawn 使用

它不是导航参数包，也不是传感器驱动包。  
LiDAR 安装外参、底盘公共几何、Gazebo workaround 都在这里维护。

## 包内结构

```text
src/sentry_robot_description/
├── launch/
│   └── robot_description_launch.py
├── params/
│   └── robot_description.yaml
├── resource/
│   └── xmacro/
│       ├── wheeled_biped_core.sdf.xmacro
│       ├── wheeled_biped_real.sdf.xmacro
│       ├── wheeled_biped_sim.sdf.xmacro
│       └── wheeled_biped.sdf.xmacro
├── rviz/
│   └── visualize_robot.rviz
└── env-hooks/
    └── gazebo.dsv.in
```

## 当前模型设计

当前只维护一套 2026 差速轮足模型：`wheeled_biped`。

TF 链固定为：

```text
map -> odom -> base_footprint -> chassis -> gimbal_yaw -> gimbal_pitch -> front_mid360
```

语义约束：

- `base_footprint` 是导航使用的 2D 基座
- `chassis` 与 `base_footprint` 共享 yaw
- 云台 frame 名固定为 `gimbal_yaw -> gimbal_pitch`
- Mid360 和工业相机都挂在 `gimbal_pitch`
- LiDAR 斜装、倒装、下俯都只能写在 `gimbal_pitch -> front_mid360`
- 不能把 LiDAR 安装角写进 Point-LIO 的 `gravity`

当前默认语义：

- 实车 profile: `chassis -> gimbal_yaw -> gimbal_pitch` 是静态 TF
- 仿真 profile: `gimbal_yaw_joint / gimbal_pitch_joint` 仍是动态关节

## 三个 xmacro 分别干什么

### 1. `wheeled_biped_core.sdf.xmacro`

共享核心，不直接作为入口使用。

这里放的是：

- 公共 link / joint / frame 名字
- 左右驱动轮
- `base_footprint -> chassis -> gimbal_yaw -> gimbal_pitch` 主体拓扑
- 固定云台 / 动态云台两种 joint 语义开关（`use_fixed_gimbal`）
- Mid360 / industrial camera 的公共 block
- Gazebo caster / plugin 的可复用 block

只有“实车和仿真都应该一起变”的结构变化，才改这里。

典型例子：

- 轮距变了
- 轮半径变了
- 云台高度变了
- link / joint 名字要变
- TF 拓扑要变

### 2. `wheeled_biped_real.sdf.xmacro`

实车入口。

默认被下面两个 launch 使用：

- `ros2 launch sentry_robot_description robot_description_launch.py`
- `ros2 launch sentry_nav_bringup robot_state_publisher_launch.py`

这里应该只保留实车真实语义：

- 实车 chassis 尺寸
- 实车 LiDAR 外参（当前默认 15° 下俯）
- 实车相机外参
- 实车固定云台 TF
- TF / visual / collision

这里**不应该**出现：

- Gazebo caster
- Gazebo DiffDrive plugin
- Gazebo `sensor`
- 仿真专用 30° 下俯角
- 仿真专用 65% chassis 缩小
- 串口 joint state 驱动的动态云台 TF

### 3. `wheeled_biped_sim.sdf.xmacro`

Gazebo 入口。

默认被下面的仿真 spawn 使用：

- `ros2 launch rmu_gazebo_simulator bringup_sim.launch.py`

这里放仿真专用 workaround：

- 65% chassis box，减少自遮挡
- 前后 caster ball，替代实车主动平衡控制
- Gazebo DiffDrive plugin
- Gazebo gimbal joint controller
- `gpu_lidar` / camera / chassis imu sensor
- Mid360 固定下俯约 30°，补近场低矮底座感知
- 动态 `gimbal_yaw_joint / gimbal_pitch_joint`

### 4. `wheeled_biped.sdf.xmacro`

兼容别名。

当前等价于 `wheeled_biped_real.sdf.xmacro`，只为了兼容旧引用。

新开发不要再把它当主文件改。

## 你平时该改哪个文件

| 需求 | 改哪个文件 | 典型参数 |
|---|---|---|
| 实车 LiDAR 斜装、倒装、实测平移变化 | `wheeled_biped_real.sdf.xmacro` | `front_lidar_pose` |
| 实车云台改成固定安装 | `wheeled_biped_real.sdf.xmacro` | `use_fixed_gimbal=True`（默认已开启） |
| 仿真里想调下俯角、仿真 chassis 尺寸 | `wheeled_biped_sim.sdf.xmacro` | `front_lidar_pose`、`chassis_length/width/z` |
| 公共轮距、轮径、云台高度变化 | `wheeled_biped_core.sdf.xmacro` | 公共 joint / 几何参数 |

快速判断：

- 改完后只应该影响实车 TF：改 `real`
- 改完后只应该影响 Gazebo：改 `sim`
- 改完后实车和仿真都应该一起变：改 `core`

## 当前默认参数

### 实车入口默认值

文件：`wheeled_biped_real.sdf.xmacro`

- `chassis_length = 0.648`
- `chassis_width = 0.650`
- `chassis_z = 0.120`
- `front_lidar_pose = -0.05 0 0.05 0.0 0.2617993877991494 3.141592653589793`
- `use_fixed_gimbal = True`

说明：

- 当前保留了 Mid360 的 yaw 反向安装
- 当前实车默认是 15° 下俯
- 实车 `gimbal_yaw_joint / gimbal_pitch_joint` 为 fixed，不再依赖串口 joint state 驱动 TF

### 仿真入口默认值

文件：`wheeled_biped_sim.sdf.xmacro`

- `chassis_length = 0.4212`
- `chassis_width = 0.4225`
- `chassis_z = 0.078`
- `front_lidar_pose = -0.05 0 0.05 0.0 0.523598775598299 3.141592653589793`
- `use_fixed_gimbal = False`

说明：

- chassis 缩到实车的 65%
- Mid360 固定下俯约 30°
- 云台关节保持 Gazebo 动态控制

## 和 gravity 的边界

这个包只负责**几何外参 / TF 语义**。

它不负责 Point-LIO 的重力状态初始化。

必须区分：

- LiDAR 安装角：改这里的 `front_lidar_pose`
- Point-LIO `gravity / gravity_init`：改导航参数或用工具箱标定

不要把 LiDAR 安装角手算进 `gravity`。  
那会把传感器几何问题和 IMU 重力方向问题混在一起。

## 外参与 DOF 怎么理解

`front_lidar_pose` 当前使用：

```text
x y z roll pitch yaw
```

含义是：

- `x y z`: 子坐标系原点在父坐标系里的平移，单位米
- `roll`: 绕父坐标系 `x` 轴转，单位弧度
- `pitch`: 绕父坐标系 `y` 轴转，单位弧度
- `yaw`: 绕父坐标系 `z` 轴转，单位弧度

在当前模型里：

- `front_lidar_pose` 永远表示 `gimbal_pitch -> front_mid360`
- 也就是“雷达相对云台安装板”的外参
- 不要把它理解成 `map`、`odom` 或 `base_footprint` 下的绝对姿态

### 当前 TF / joint 的 DOF 语义

当前实车默认：

```text
base_footprint -> chassis         fixed
chassis -> gimbal_yaw             fixed
gimbal_yaw -> gimbal_pitch        fixed
gimbal_pitch -> front_mid360      fixed (由 front_lidar_pose 定义)
```

所以当前实车有效语义是：

- 底盘和云台一起运动，是同一个刚体链
- LiDAR 只有“安装外参”这一个固定姿态差
- 改 LiDAR 安装角，只改 `front_lidar_pose`
- 不要去改 `gravity` 表达 LiDAR 安装角

当前仿真默认：

```text
base_footprint -> chassis         fixed
chassis -> gimbal_yaw             revolute
gimbal_yaw -> gimbal_pitch        revolute
gimbal_pitch -> front_mid360      fixed
```

所以仿真保留了云台动态 DOF，实车默认没有。

### roll / pitch / yaw 正方向

按 ROS 常用右手系：

- `+roll`: 绕 `x` 轴正向旋转
- `+pitch`: 绕 `y` 轴正向旋转
- `+yaw`: 绕 `z` 轴正向旋转

在这个包当前 Mid360 安装语义里，最常用的是：

- `pitch > 0`: 雷达下俯
- `pitch < 0`: 雷达上仰
- `yaw = pi`: 雷达前后反装 180°

当前实车 15° 下俯即：

- `pitch = +0.2617993877991494`

`front_lidar_pose` 当前使用 `x y z roll pitch yaw`。

- `pitch > 0` 表示雷达下俯
- `pitch < 0` 表示雷达上仰
- 当前实车 15° 下俯即 `pitch = 0.2617993877991494`

## 角度为什么是 0.261799...

SDF / URDF / TF 的旋转默认都用**弧度**，不是角度。

换算公式：

```text
radian = degree * pi / 180
degree = radian * 180 / pi
```

所以：

- `15° = 15 * pi / 180 = pi / 12 = 0.2617993877991494`
- `30° = 0.5235987755982988`
- `45° = 0.7853981633974483`
- `90° = 1.5707963267948966`
- `180° = 3.141592653589793`

常用心算：

- 15° 看成 `pi / 12`
- 30° 看成 `pi / 6`
- 45° 看成 `pi / 4`
- 90° 看成 `pi / 2`
- 180° 看成 `pi`

如果只是临时算一个角度：

```bash
python3 -c "import math; print(math.radians(15))"
python3 -c "import math; print(math.degrees(0.2617993877991494))"
```

## 常见安装场景怎么写

以下都只讨论 `front_lidar_pose` 的 `roll pitch yaw` 三个角。

### 1. 正装，只是下俯 15°

```text
0.0 0.2617993877991494 0.0
```

也就是当前实车这种情况。

### 2. 正装，只是上仰 15°

```text
0.0 -0.2617993877991494 0.0
```

### 3. 正装，但左右歪了 10°

```text
0.0 0.0 0.17453292519943295
```

也就是只改 `yaw`。

### 4. 雷达倒置

最常见先从 `roll = pi` 或 `yaw = pi` 去理解。

具体写法要看你是绕哪根机械轴翻过去的，但原则是：

- 倒置属于**安装姿态**
- 仍然只改 `front_lidar_pose`
- 不改 `gravity`

常见例子：

- 若雷达是“上下翻转”装上去，通常先试 `roll = pi`
- 若雷达是“前后反装”装上去，通常先试 `yaw = pi`

当前模型历史上保留了：

```text
yaw = 3.141592653589793
```

这就是一个 180° 反装。

### 5. 倒置后再下俯 15°

这类场景不要只凭脑补改一个角。

正确做法：

1. 先确定基础倒置是绕哪根轴翻的
2. 再在那个基础姿态上叠加俯仰角
3. 用 RViz / TF 树看 `front_mid360` 坐标轴是否符合实物

如果不确定，宁可分两步验证：

1. 先只写倒置
2. 看坐标轴方向
3. 再叠加 `pitch`

## 改外参时最容易犯的错

- 把角度直接写进 SDF，忘了要用弧度
- 把 LiDAR 安装角写进 Point-LIO `gravity`
- 把 `front_lidar_pose` 误当成世界坐标系姿态
- 底盘/云台已经固定，却还去改 `gimbal_yaw_joint` 或 `gimbal_pitch_joint` 的动态关节语义
- 倒装时一次同时改多个轴，最后分不清是谁起作用

最稳的流程是：

1. 先只改一个轴
2. 用弧度写值
3. 启动 `robot_state_publisher`
4. 在 RViz 检查 `front_mid360` 的坐标轴方向
5. 确认对了再继续叠加下一个角

## Launch 用法

### 1. 查看默认实车描述

```bash
ros2 launch sentry_robot_description robot_description_launch.py
```

默认等价于：

```bash
ros2 launch sentry_robot_description robot_description_launch.py robot_name:=wheeled_biped_real
```

### 2. 直接查看仿真描述

```bash
ros2 launch sentry_robot_description robot_description_launch.py robot_name:=wheeled_biped_sim
```

### 3. 指定绝对路径入口

```bash
ros2 launch sentry_robot_description robot_description_launch.py \
  robot_xmacro_file:=/abs/path/to/custom_model.sdf.xmacro
```

## `robot_description_launch.py` 做了什么

这个 launch 会：

1. 读取 `robot_name` 或 `robot_xmacro_file`
2. 用 `xmacro4sdf` 风格解析 xmacro，生成 SDF
3. 用 `sdformat_tools` 把 SDF 转成 URDF
4. 启动：
   - 可选 `joint_state_publisher`
   - `robot_state_publisher`
   - 可选 `rviz2`

默认参数：

- `robot_name = wheeled_biped_real`
- `use_sim_time = False`
- `use_rviz = True`
- `use_joint_state_publisher = False`

补充说明：

- 默认 `params/robot_description.yaml` 的 `source_list` 为空，因为当前实车 profile 使用静态云台 TF
- `serial/gimbal_joint_state` 仍会保留给调试/可视化使用，但默认不再驱动 TF
- 如果后续恢复动态云台，可传 `use_joint_state_publisher:=True` 并提供带 `source_list` 的参数文件

## 环境变量

`env-hooks/gazebo.dsv.in` 会自动把模型路径导出到：

- `GZ_SIM_RESOURCE_PATH`
- `IGN_GAZEBO_RESOURCE_PATH`
- `SDF_PATH`
- `GZ_FILE_PATH`

这样 xmacro 的 `model://mid360/...`、Gazebo 的模型资源和 mesh 才能被正确解析。

## 常用验证

### 1. 验证 launch 默认入口

```bash
ros2 launch sentry_robot_description robot_description_launch.py --show-args
```

应看到默认：

```text
robot_name: wheeled_biped_real
```

### 2. 验证 xmacro 能展开

```bash
xmacro4sdf src/sentry_robot_description/resource/xmacro/wheeled_biped_real.sdf.xmacro > /tmp/wheeled_biped_real.sdf
xmacro4sdf src/sentry_robot_description/resource/xmacro/wheeled_biped_sim.sdf.xmacro > /tmp/wheeled_biped_sim.sdf
```

### 3. 验证包能编译

```bash
colcon build --packages-select sentry_robot_description --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 当前为什么不改包名

`sentry_robot_description` 现在这个包名仍然合适。

原因很简单：

- 这是 ROS 里标准的职责命名
- 它确实就是机器人描述包
- 现在已经被多个 launch / package / 文档直接依赖

现在改名收益很小，但改动面会很大：

- `get_package_share_directory("sentry_robot_description")`
- 依赖包 `package.xml`
- launch 引用
- 文档路径
- 安装路径

所以现阶段不建议为了“名字更具体”去重命名它。

如果未来模型完全稳定，确实要更具体，可以再统一改成类似：

- `sentry_wheeled_biped_description`

但那应该是一次专门的重命名工作，不应和当前调参与仿真修复混做。
