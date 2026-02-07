# YOPO 硬件集成配置

## 系统架构

```
[硬件层]
    RealSense D455f ─────┬─── 深度图 ──────────────────────────┐
                         └─── 双目红外图 ───┐                  │
                                            │                  │
    PX4 飞控 (MAVROS) ───── IMU ───────────┼──► VINS-Fusion   │
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

> **当前配置**: 使用 PX4 飞控 IMU，D455f 仅提供双目红外图像和深度图。

---

## 强制约束

| ID | 约束内容 | 原因 |
|----|----------|------|
| C-001 | VINS-Fusion **必须**使用 PX4 IMU (`/mavros/imu/data_raw`) | 统一使用飞控 IMU，简化系统 |
| C-002 | VINS-Fusion **必须**使用双目红外图像 | 红外图像不受环境光影响，纹理更稳定 |
| C-003 | 里程计**必须**使用 NWU 坐标系 | YOPO 训练数据使用 NWU |
| C-004 | 深度图**必须** 480×270 (16:9) | 神经网络输入维度固定 |
| C-005 | D455f IMU **必须**禁用 (enable_gyro/accel: false) | 避免带宽占用和话题混淆 |

---

## RealSense D455f 配置

### 发布话题

| 话题 | 类型 | 频率 | 订阅者 |
|------|------|------|--------|
| `/camera/depth/image_rect_raw` | sensor_msgs/Image | 30Hz | YOPO |
| `/camera/infra1/image_rect_raw` | sensor_msgs/Image | 30Hz | VINS |
| `/camera/infra2/image_rect_raw` | sensor_msgs/Image | 30Hz | VINS |

> **注意**: D455f IMU 已禁用，不会有 `/camera/imu` 话题。IMU 数据由 MAVROS 提供。

### YOPO 专用参数

```yaml
depth_width: 480
depth_height: 270
depth_fps: 30
enable_infra1: true
enable_infra2: true
infra_width: 640
infra_height: 480
enable_gyro: false   # 禁用 D455f IMU，使用 PX4 IMU
enable_accel: false  # 禁用 D455f IMU，使用 PX4 IMU
enable_color: false  # 实飞时关闭
```

### 深度图格式

| 属性 | 值 |
|------|-----|
| 编码 | 16UC1 |
| 分辨率 | 480 × 270 |
| 最大深度 | 20.0 米 |

---

## VINS-Fusion 配置

**配置文件**: `/home/amov/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml`

### 订阅话题

| 话题 | 来源 |
|------|------|
| `/camera/infra1/image_rect_raw` | D455f |
| `/camera/infra2/image_rect_raw` | D455f |
| `/mavros/imu/data_raw` | PX4 飞控 (MAVROS) |

### 发布话题

| 话题 | 频率 | 订阅者 |
|------|------|--------|
| `/vins_estimator/imu_propagate` | 100Hz+ | YOPO, SO3 控制器 |

### 里程计消息格式

```yaml
header:
  stamp: <ros::Time>
  frame_id: "world"
pose:
  pose:
    position: {x, y, z}           # 单位: 米
    orientation: {x, y, z, w}     # NWU 坐标系四元数
twist:
  twist:
    linear: {x, y, z}             # 单位: m/s
```

---

## YOPO 规划器配置

```python
settings = {
    'odom_topic': '/vins_estimator/imu_propagate',
    'depth_topic': '/camera/depth/image_rect_raw',
    'ctrl_topic': '/so3_control/pos_cmd',
    'use_tensorrt': 1,
    'plan_from_reference': True,
    'visualize': False,  # 实飞时必须关闭
}
```

### 输出话题

| 话题 | 类型 | 频率 |
|------|------|------|
| `/so3_control/pos_cmd` | quadrotor_msgs/PositionCommand | 50Hz |

---

## SO3 控制器配置

**Launch 文件**: `/home/amov/Projects/YOPO/Controller/src/so3_control/launch/controller_network.launch`

```xml
<node pkg="so3_control" type="network_control_node" name="network_controller_node">
    <param name="is_simulation" value="false"/>
    <param name="use_disturbance_observer" value="true"/>
    <param name="hover_thrust" value="0.38"/>  <!-- 需实际标定 -->
    <remap from="~odom" to="/vins_estimator/imu_propagate"/>
    <remap from="~imu" to="/mavros/imu/data_raw"/>
    <remap from="~position_cmd" to="/so3_control/pos_cmd"/>
</node>
```

---

## MAVROS 配置

### 发布话题

| 话题 | 类型 | 订阅者 |
|------|------|--------|
| `/mavros/state` | mavros_msgs/State | SO3 控制器 |
| `/mavros/imu/data_raw` | sensor_msgs/Imu | SO3 控制器 |

### 订阅话题

| 话题 | 类型 | 发布者 |
|------|------|--------|
| `/mavros/setpoint_raw/attitude` | mavros_msgs/AttitudeTarget | SO3 控制器 |

### 服务接口

| 服务 | 用途 |
|------|------|
| `/mavros/cmd/arming` | 电机解锁/上锁 |
| `/mavros/set_mode` | 设置 OFFBOARD 模式 |

---

## 时序约束

| 约束 | 要求值 |
|------|--------|
| 里程计发布频率 | ≥ 100Hz |
| 深度图发布频率 | 30Hz |
| 控制指令频率 | 50Hz |
| YOPO 推理延迟 | < 5ms |
| 端到端延迟 | < 30ms |

---

## 故障检测

| 故障类型 | 检测条件 | 处理方式 |
|----------|----------|----------|
| 里程计丢失 | 超过 100ms 无更新 | 悬停/降落 |
| 深度图丢失 | 超过 200ms 无更新 | 保持当前轨迹 |
| VIO 发散 | 位置跳变 > 1m | 停止规划 |

---

## 话题频率检查命令

```bash
# 深度图 (应为 30Hz)
rostopic hz /camera/depth/image_rect_raw

# 里程计 (应 >= 100Hz)
rostopic hz /vins_estimator/imu_propagate

# 控制指令 (应为 50Hz)
rostopic hz /so3_control/pos_cmd
```
