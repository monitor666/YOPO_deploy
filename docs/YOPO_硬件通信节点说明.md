# YOPO 无人机自主导航系统 - 硬件通信接口规范

> **文档用途**: 本文档定义了 YOPO 项目实机部署时的硬件通信节点规范。AI 助手在处理 YOPO 相关任务时，必须遵循本文档中的约束条件。

---

## 元信息

| 属性 | 值 |
|------|-----|
| 项目名称 | YOPO (You Only Plan Once) |
| 文档类型 | 硬件通信接口规范 |
| 适用场景 | 实机部署 |
| 硬件配置 | PX4 飞控 + Intel RealSense D455f |
| VIO 方案 | VINS-Fusion |
| IMU 来源 | **PX4 飞控 IMU** (通过 MAVROS) |

---

## 系统数据流架构

```
[硬件层]
    RealSense D455f ─────┬─── 深度图 ──────────────────────────┐
                         └─── 双目红外图 ───┐                  │
                                           │                  │
    PX4 飞控 ──► MAVROS ─── IMU ───────────┼──► VINS-Fusion   │
                                           │        │          │
                                           └────────┼──────────┤
                                                    │          │
                                    里程计 ◄────────┘          │
                                       │                       │
                                       ▼                       ▼
[规划层]                          YOPO 规划器 ◄────────────────┘
                                       │
                                       ▼ 轨迹指令
[控制层]                          SO3 控制器
                                       │
                                       ▼ 姿态指令
[飞控层]                     MAVROS ──► PX4 飞控
```

---

## IMU 方案说明

> **当前配置**: 使用 **PX4 飞控 IMU** 作为 VINS-Fusion 输入。

### 方案对比

| 项目 | D455f IMU 方案 | PX4 IMU 方案 (当前) |
|------|----------------|---------------------|
| IMU 来源 | 相机内置 | 飞控 |
| ROS 话题 | /camera/imu | /mavros/imu/data_raw |
| 时间同步 | 硬件同步 | 软件同步 (estimate_td) |
| 外参标定 | 出厂标定 | 需重新标定 |
| IMU 位置 | 相机上 | 机体重心附近 |

### 当前方案优势

1. PX4 IMU 更接近机体重心，外参标定更直观
2. 节省 D455f 带宽，相机只输出图像流
3. 控制器和 VIO 使用同一 IMU 源，数据一致性更好

### 注意事项

- **必须先启动 MAVROS** 再启动 VINS-Fusion
- 需要标定 PX4 IMU 到相机的外参
- VINS-Fusion 的 `estimate_td` 参数会在线估计时间偏移

---

## 约束条件

### 强制约束 (MUST)

以下约束条件在任何情况下都必须遵守：

| ID | 约束内容 |
|----|----------|
| C-001 | VINS-Fusion 的 IMU 输入**必须**使用 PX4 飞控 IMU (话题: `/mavros/imu/data_raw`) |
| C-002 | VINS-Fusion 的图像输入**必须**使用 D455f 的双目红外图像 |
| C-003 | 所有里程计数据**必须**使用 NWU (北-西-上) 坐标系 |
| C-004 | 深度图分辨率**必须**保持 16:9 宽高比 |
| C-005 | YOPO 规划器**必须**同时订阅深度图和里程计两个话题 |
| C-006 | D455f 内置 IMU **必须**禁用 (enable_gyro/accel: false) |
| C-007 | MAVROS **必须**在 VINS-Fusion 之前启动 |

### 约束原因

| 约束 ID | 原因说明 |
|---------|----------|
| C-001 | PX4 IMU 更接近机体重心，外参标定更直观；控制器与 VIO 使用同一 IMU 源 |
| C-002 | 红外图像不受环境光影响，纹理更稳定，且与深度图时间戳同步 |
| C-003 | YOPO 训练数据使用 NWU 坐标系，坐标系不匹配会导致规划失败 |
| C-004 | 神经网络输入维度固定，非 16:9 比例会导致畸变 |
| C-005 | 缺少任一输入会导致规划器无法工作 |
| C-006 | 避免 D455f IMU 占用带宽，且防止话题混淆 |
| C-007 | VINS-Fusion 需要 IMU 数据才能正常初始化 |

---

## 硬件输入节点定义

### 节点 1: MAVROS (PX4 飞控接口)

**ROS 包**: `mavros`

> **重要**: MAVROS 必须最先启动，它提供 VIO 所需的 IMU 数据。

#### 发布话题 (供其他节点订阅)

| 话题名称 | 消息类型 | 频率 | 目标订阅者 | 必需 |
|----------|----------|------|------------|------|
| `/mavros/imu/data_raw` | `sensor_msgs/Imu` | 200Hz+ | **VINS-Fusion**, SO3 控制器 | ✅ 是 |
| `/mavros/state` | `mavros_msgs/State` | 1Hz | SO3 控制器 | ✅ 是 |

#### 订阅话题 (接收控制指令)

| 话题名称 | 消息类型 | 用途 | 发布者 |
|----------|----------|------|--------|
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | 姿态+推力 | SO3 控制器 |

#### 服务接口

| 服务名称 | 消息类型 | 用途 |
|----------|----------|------|
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | 电机解锁/上锁 |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | 设置 OFFBOARD 模式 |

#### 启动命令

```bash
# USB 连接
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600

# 串口连接 (Jetson UART)
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS0:921600
```

#### IMU 200Hz 配置 (重要)

MAVROS 默认请求的 IMU 频率约 50Hz，需要配置为 200Hz。

**每次启动后手动配置 (临时)**

```bash
conda deactivate
bash ~/Projects/scripts/set_mavros_imu_rate.sh
```

**验证**: `rostopic hz /mavros/imu/data_raw` (应显示 ~200Hz)

---

### 节点 2: RealSense D455f 驱动

**ROS 包**: `realsense2_camera`

> **注意**: D455f 内置 IMU 已禁用，仅输出图像流。

#### 发布话题

| 话题名称 | 消息类型 | 频率 | 目标订阅者 | 必需 |
|----------|----------|------|------------|------|
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 30Hz | YOPO 规划器 | ✅ 是 |
| `/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | 30Hz | VINS-Fusion | ✅ 是 |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | 30Hz | VINS-Fusion | ✅ 是 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 30Hz | (可选可视化) | ❌ 否 |

> **注意**: 由于 D455f IMU 已禁用，不会有 `/camera/imu` 话题。

#### 配置参数

```yaml
# 深度相机配置 (必须)
depth_width: 480
depth_height: 270
depth_fps: 30
enable_depth: true
depth_format: "Z16"

# 红外双目配置 (必须 - 用于 VINS-Fusion)
enable_infra1: true
enable_infra2: true
infra_width: 640
infra_height: 480
infra_fps: 30

# IMU 配置 (禁用 - 使用 PX4 IMU)
enable_gyro: false
enable_accel: false

# 可选配置
enable_color: false  # 实飞时关闭节省带宽
```

#### 深度图格式要求

| 属性 | 要求值 | 说明 |
|------|--------|------|
| 编码 | `16UC1` | RealSense 原生格式，单位毫米 |
| 分辨率 | 480 × 270 | 16:9 宽高比 |
| 最大深度 | 20.0 米 | 超出范围视为无效 |
| FOV | 水平 90°, 垂直 60° | 与训练配置一致 |

#### 启动命令

```bash
# 使用 YOPO 定制版 launch (推荐)
roslaunch ~/Projects/configs/realsense/yopo_d455f_camera.launch
```

---

### 节点 3: VINS-Fusion (VIO 里程计)

**ROS 包**: `vins`

#### 订阅话题

| 话题名称 | 消息类型 | 来源 |
|----------|----------|------|
| `/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | D455f |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | D455f |
| `/mavros/imu/data_raw` | `sensor_msgs/Imu` | **PX4 (MAVROS)** |

#### 发布话题

| 话题名称 | 消息类型 | 频率 | 目标订阅者 | 必需 |
|----------|----------|------|------------|------|
| `/vins_estimator/imu_propagate` | `nav_msgs/Odometry` | 100Hz+ | YOPO, SO3控制器 | ✅ 是 |
| `/vins_estimator/odometry` | `nav_msgs/Odometry` | 10Hz | (备用) | ❌ 否 |

#### 里程计消息格式要求

```yaml
# nav_msgs/Odometry 必须包含以下字段

header:
  stamp: <ros::Time>       # 必须: 时间戳
  frame_id: "world"        # 必须: 世界坐标系

pose:
  pose:
    position:              # 必须: 位置 (单位: 米)
      x: <float64>
      y: <float64>
      z: <float64>
    orientation:           # 必须: 姿态四元数 (NWU 坐标系)
      x: <float64>
      y: <float64>
      z: <float64>
      w: <float64>

twist:
  twist:
    linear:                # 必须: 线速度 (单位: m/s)
      x: <float64>
      y: <float64>
      z: <float64>
```

#### PX4 IMU 相关配置

```yaml
# VINS-Fusion 配置文件中的关键参数

imu_topic: "/mavros/imu/data_raw"   # PX4 IMU 话题

# 外参 (需要根据实际安装位置标定)
estimate_extrinsic: 1               # 在线优化外参

# 时间偏移 (PX4 IMU 与相机无硬件同步)
estimate_td: 1                      # 在线估计时间偏移
td: 0.0                             # 初始值 (可调整 -0.02 ~ 0.02)
```

---

## 节点间通信关系矩阵

| 发布节点 | 话题 | 订阅节点 |
|----------|------|----------|
| MAVROS (PX4) | `/mavros/imu/data_raw` | **VINS-Fusion**, SO3 控制器 |
| MAVROS (PX4) | `/mavros/state` | SO3 控制器 |
| D455f | `/camera/depth/image_rect_raw` | YOPO 规划器 |
| D455f | `/camera/infra1/image_rect_raw` | VINS-Fusion |
| D455f | `/camera/infra2/image_rect_raw` | VINS-Fusion |
| VINS-Fusion | `/vins_estimator/imu_propagate` | YOPO 规划器, SO3 控制器 |
| YOPO 规划器 | `/so3_control/pos_cmd` | SO3 控制器 |
| SO3 控制器 | `/mavros/setpoint_raw/attitude` | MAVROS |

---

## YOPO 规划器配置

### 输入话题配置

```python
# test_yopo_ros.py 中的 settings 配置
settings = {
    'odom_topic': '/vins_estimator/imu_propagate',   # 里程计话题
    'depth_topic': '/camera/depth/image_rect_raw',   # 深度图话题
    'ctrl_topic': '/so3_control/pos_cmd',            # 控制指令话题
    'use_tensorrt': 1,                               # 启用 TensorRT 加速
    'plan_from_reference': True,                     # 位置控制器模式
    'visualize': False,                              # 实飞关闭节省计算
}
```

### 输出话题

| 话题名称 | 消息类型 | 频率 | 用途 |
|----------|----------|------|------|
| `/so3_control/pos_cmd` | `quadrotor_msgs/PositionCommand` | 50Hz | 轨迹控制指令 |

---

## SO3 控制器配置

### Launch 文件配置

```xml
<node pkg="so3_control" type="network_control_node" name="network_controller_node">
    <param name="is_simulation" value="false"/>
    <param name="use_disturbance_observer" value="true"/>
    <param name="hover_thrust" value="0.38"/>  <!-- 需实际标定 -->
    
    <!-- 话题重映射 -->
    <remap from="~odom" to="/vins_estimator/imu_propagate"/>
    <remap from="~imu" to="/mavros/imu/data_raw"/>
    <remap from="~position_cmd" to="/so3_control/pos_cmd"/>
</node>
```

---

## 时序约束

| 约束 | 要求值 | 说明 |
|------|--------|------|
| PX4 IMU 发布频率 | ≥ 200Hz | MAVROS 发布 |
| 里程计发布频率 | ≥ 100Hz | 使用 `imu_propagate` 话题 |
| 深度图发布频率 | 30Hz | 与相机帧率一致 |
| 控制指令频率 | 50Hz | 控制周期 20ms |
| YOPO 推理延迟 | < 5ms | 使用 TensorRT |
| 端到端延迟 | < 30ms | 深度图到控制指令 |

---

## 启动顺序

> **重要**: 使用 PX4 IMU 方案时，必须按以下顺序启动：

1. **roscore**
2. **MAVROS** (提供 PX4 IMU 数据)
3. RealSense D455f (提供图像)
4. VINS-Fusion (需要 IMU + 图像)
5. SO3 控制器
6. YOPO 规划器

---

## 故障检测条件

| 故障类型 | 检测条件 | 处理方式 |
|----------|----------|----------|
| IMU 数据丢失 | `/mavros/imu/data_raw` 超过 50ms 无更新 | 停止 VIO |
| 里程计丢失 | 超过 100ms 无更新 | 悬停/降落 |
| 深度图丢失 | 超过 200ms 无更新 | 保持当前轨迹 |
| VIO 发散 | 位置跳变 > 1m | 停止规划 |
| 推理超时 | 处理时间 > 33ms | 打印警告 |

---

## 部署检查清单

在启动系统前，验证以下条件：

```
□ MAVROS (PX4)
  □ roslaunch mavros px4.launch
  □ /mavros/state 显示 connected: true
  □ /mavros/imu/data_raw 正常发布 (200Hz+)

□ RealSense D455f
  □ /camera/depth/image_rect_raw 正常发布 (30Hz)
  □ /camera/infra1/image_rect_raw 正常发布 (30Hz)
  □ /camera/infra2/image_rect_raw 正常发布 (30Hz)
  □ 确认没有 /camera/imu 话题 (IMU 已禁用)

□ VINS-Fusion
  □ 订阅 D455f 的红外图像
  □ 订阅 PX4 IMU (/mavros/imu/data_raw)
  □ /vins_estimator/imu_propagate 正常发布 (100Hz+)
  □ 输出坐标系为 NWU

□ SO3 控制器
  □ odom 重映射到 /vins_estimator/imu_propagate
  □ imu 重映射到 /mavros/imu/data_raw
  □ hover_thrust 已标定

□ YOPO 规划器
  □ odom_topic 设为 /vins_estimator/imu_propagate
  □ depth_topic 设为 /camera/depth/image_rect_raw
  □ TensorRT 模型已加载
```

---

## 禁止操作

以下操作在 YOPO 实机部署中被明确禁止：

| 禁止操作 | 原因 |
|----------|------|
| ❌ 使用 D455f IMU 作为 VINS-Fusion 输入 | 当前方案使用 PX4 IMU |
| ❌ 同时启用 D455f IMU 和 PX4 IMU | 造成数据混淆 |
| ❌ 使用 RGB 图像作为 VINS 输入 | 光照敏感，不如红外稳定 |
| ❌ 修改深度图为非 16:9 比例 | 神经网络输入维度不匹配 |
| ❌ 在 ENU 坐标系下运行 | 与训练数据坐标系不一致 |
| ❌ 实飞时开启 visualize=True | 增加计算负担导致延迟 |
| ❌ 在 MAVROS 之前启动 VINS-Fusion | VIO 无法获取 IMU 数据 |

---

## 参考资料

- [YOPO GitHub 仓库](https://github.com/TJU-Aerial-Robotics/YOPO)
- [VINS-Fusion GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [RealSense ROS 文档](https://github.com/IntelRealSense/realsense-ros)
- [MAVROS 文档](http://wiki.ros.org/mavros)
- [PX4 用户指南](https://docs.px4.io/)