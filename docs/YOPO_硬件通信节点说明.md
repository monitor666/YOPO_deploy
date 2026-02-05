# YOPO 无人机自主导航系统 - 硬件通信接口规范

> **文档用途**: 本文档定义了 YOPO 项目实机部署时的硬件通信节点规范。AI 助手在处理 YOPO 相关任务时，必须遵循本文档中的约束条件。

---

## 元信息

| 属性 | 值 |
|------|-----|
| 项目名称 | YOPO (You Only Plan Once) |
| 文档类型 | 硬件通信接口规范 |
| 适用场景 | 实机部署 |
| 硬件配置 | PX4 飞控 + Intel RealSense D435i |
| VIO 方案 | VINS-Fusion |

---

## 系统数据流架构

```
[硬件层]
    RealSense D435i ─────┬─── 深度图 ──────────────────────────┐
                         ├─── 双目红外图 ───┐                  │
                         └─── IMU ─────────┼──► VINS-Fusion   │
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

## 约束条件

### 强制约束 (MUST)

以下约束条件在任何情况下都必须遵守：

| ID | 约束内容 |
|----|----------|
| C-001 | VINS-Fusion 的 IMU 输入**必须**使用 D435i 自带的 IMU，**禁止**使用 PX4 飞控的 IMU |
| C-002 | VINS-Fusion 的图像输入**必须**使用 D435i 的双目红外图像 |
| C-003 | 所有里程计数据**必须**使用 NWU (北-西-上) 坐标系 |
| C-004 | 深度图分辨率**必须**保持 16:9 宽高比 |
| C-005 | YOPO 规划器**必须**同时订阅深度图和里程计两个话题 |

### 约束原因

| 约束 ID | 原因说明 |
|---------|----------|
| C-001 | D435i 内部已完成 IMU-相机硬件时间同步，使用外部 IMU 会导致 VIO 发散 |
| C-002 | 红外图像不受环境光影响，纹理更稳定，且与深度图时间戳同步 |
| C-003 | YOPO 训练数据使用 NWU 坐标系，坐标系不匹配会导致规划失败 |
| C-004 | 神经网络输入维度固定，非 16:9 比例会导致畸变 |
| C-005 | 缺少任一输入会导致规划器无法工作 |

---

## 硬件输入节点定义

### 节点 1: RealSense D435i 驱动

**ROS 包**: `realsense2_camera`

#### 发布话题

| 话题名称 | 消息类型 | 频率 | 目标订阅者 | 必需 |
|----------|----------|------|------------|------|
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 30Hz | YOPO 规划器 | ✅ 是 |
| `/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | 30Hz | VINS-Fusion | ✅ 是 |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | 30Hz | VINS-Fusion | ✅ 是 |
| `/camera/imu` | `sensor_msgs/Imu` | 200Hz | VINS-Fusion | ✅ 是 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 30Hz | (可选可视化) | ❌ 否 |

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

# IMU 配置 (必须 - 用于 VINS-Fusion)
enable_gyro: true
enable_accel: true
unite_imu_method: "linear_interpolation"
gyro_fps: 200
accel_fps: 200

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

---

### 节点 2: VINS-Fusion (VIO 里程计)

**ROS 包**: `vins`

#### 订阅话题

| 话题名称 | 消息类型 | 来源 |
|----------|----------|------|
| `/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | D435i |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | D435i |
| `/camera/imu` | `sensor_msgs/Imu` | D435i |

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

---

### 节点 3: MAVROS (飞控接口)

**ROS 包**: `mavros`

#### 发布话题 (供其他节点订阅)

| 话题名称 | 消息类型 | 用途 | 订阅者 |
|----------|----------|------|--------|
| `/mavros/state` | `mavros_msgs/State` | 飞控状态 | SO3 控制器 |
| `/mavros/imu/data_raw` | `sensor_msgs/Imu` | 飞控 IMU | SO3 控制器 (扰动观测) |

#### 订阅话题 (接收控制指令)

| 话题名称 | 消息类型 | 用途 | 发布者 |
|----------|----------|------|--------|
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | 姿态+推力 | SO3 控制器 |

#### 服务接口

| 服务名称 | 消息类型 | 用途 |
|----------|----------|------|
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | 电机解锁/上锁 |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | 设置 OFFBOARD 模式 |

---

## 节点间通信关系矩阵

| 发布节点 | 话题 | 订阅节点 |
|----------|------|----------|
| D435i | `/camera/depth/image_rect_raw` | YOPO 规划器 |
| D435i | `/camera/infra1/image_rect_raw` | VINS-Fusion |
| D435i | `/camera/infra2/image_rect_raw` | VINS-Fusion |
| D435i | `/camera/imu` | VINS-Fusion |
| VINS-Fusion | `/vins_estimator/imu_propagate` | YOPO 规划器, SO3 控制器 |
| MAVROS | `/mavros/imu/data_raw` | SO3 控制器 |
| MAVROS | `/mavros/state` | SO3 控制器 |
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
| 里程计发布频率 | ≥ 100Hz | 使用 `imu_propagate` 话题 |
| 深度图发布频率 | 30Hz | 与相机帧率一致 |
| 控制指令频率 | 50Hz | 控制周期 20ms |
| YOPO 推理延迟 | < 5ms | 使用 TensorRT |
| 端到端延迟 | < 30ms | 深度图到控制指令 |

---

## 故障检测条件

| 故障类型 | 检测条件 | 处理方式 |
|----------|----------|----------|
| 里程计丢失 | 超过 100ms 无更新 | 悬停/降落 |
| 深度图丢失 | 超过 200ms 无更新 | 保持当前轨迹 |
| VIO 发散 | 位置跳变 > 1m | 停止规划 |
| 推理超时 | 处理时间 > 33ms | 打印警告 |

---

## 部署检查清单

在启动系统前，验证以下条件：

```
□ RealSense D435i
  □ /camera/depth/image_rect_raw 正常发布 (30Hz)
  □ /camera/infra1/image_rect_raw 正常发布 (30Hz)
  □ /camera/infra2/image_rect_raw 正常发布 (30Hz)
  □ /camera/imu 正常发布 (200Hz)
  □ 深度图分辨率为 480×270

□ VINS-Fusion
  □ 订阅 D435i 的红外图像和 IMU (非 PX4 IMU)
  □ /vins_estimator/imu_propagate 正常发布 (100Hz+)
  □ 输出坐标系为 NWU

□ MAVROS
  □ /mavros/state 显示 connected: true
  □ /mavros/imu/data_raw 正常发布

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
| ❌ 使用 PX4 IMU 作为 VINS-Fusion 输入 | 时间同步问题导致 VIO 发散 |
| ❌ 使用 RGB 图像作为 VINS 输入 | 光照敏感，不如红外稳定 |
| ❌ 修改深度图为非 16:9 比例 | 神经网络输入维度不匹配 |
| ❌ 在 ENU 坐标系下运行 | 与训练数据坐标系不一致 |
| ❌ 实飞时开启 visualize=True | 增加计算负担导致延迟 |

---

## 参考资料

- [YOPO GitHub 仓库](https://github.com/TJU-Aerial-Robotics/YOPO)
- [VINS-Fusion GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [RealSense ROS 文档](https://github.com/IntelRealSense/realsense-ros)
- [MAVROS 文档](http://wiki.ros.org/mavros)
