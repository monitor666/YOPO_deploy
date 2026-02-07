# YOPO 部署工作清单

> **文档用途**: YOPO 无人机自主导航系统后续部署步骤。驱动已编译安装，RealSense 相机已启动成功。

---

## 当前部署状态

### 已完成

| 项目 | 状态 | 备注 |
|------|------|------|
| librealsense 2.50.0 | ✅ 已安装 | 源码编译 |
| realsense-ros (ros1-legacy) | ✅ 已安装 | rs_camera.launch 启动成功 |
| VINS-Fusion | ✅ 已安装 | OpenCV 4.x 兼容性已修复 |
| catkin_ws 编译 | ✅ 已完成 | ~/Projects/catkin_ws |
| yopo conda 环境 | ✅ 已创建 | Python 3.8, Jetson PyTorch |
| conda hooks 隔离 | ✅ 已配置 | PYTHONPATH 优先级正确 |
| Controller 编译 | ✅ 已完成 | ~/Projects/YOPO/Controller |
| rosfix OpenCV 修复 | ✅ 已配置 | ~/.bashrc |
| 训练模型 | ✅ 已存在 | saved/YOPO_1/epoch50.pth |
| TensorRT 模型转换 | ✅ 已完成 | yopo_trt.pth |

### 待完成

| 项目 | 优先级 | 备注 |
|------|--------|------|
| ~~MAVROS 安装与配置~~ | ~~高~~ | ✅ 已完成，IMU 频率自动配置 |
| VINS-Fusion 配置验证 (PX4 IMU) | 高 | 验证 VIO 输出 |
| PX4 IMU 到相机外参标定 | 高 | kalibr 或物理测量 |
| **YOPO 规划器深度图 Resize** | 高 | 640x480 → 480x270 |
| SO3 控制器参数标定 | 中 | hover_thrust 等 |
| 系统集成测试 | 高 | 全链路验证 |

---

## IMU 方案说明

> **当前配置**: 使用 **PX4 飞控 IMU** 作为 VINS-Fusion 输入，D455f 仅提供双目红外图像和深度图。

### 方案对比

| 项目 | D455f IMU 方案 | PX4 IMU 方案 (当前) |
|------|----------------|---------------------|
| IMU 来源 | 相机内置 | 飞控 |
| 时间同步 | 硬件同步 | 软件同步 (estimate_td) |
| 外参标定 | 出厂标定/在线优化 | 需重新标定 |
| IMU 位置 | 相机上 | 机体重心附近 |

### 当前方案优势

1. PX4 IMU 更接近机体重心，外参标定更直观
2. 节省 D455f 带宽，相机只输出图像流
3. 控制器使用同一 IMU 源，数据一致性更好

### 注意事项

- **必须先启动 MAVROS** 再启动 VINS-Fusion
- 需要标定 PX4 IMU 到相机的外参
- VINS-Fusion 会通过 `estimate_td` 在线估计时间偏移

---

## 第一阶段: MAVROS 安装与配置

### 1.1 安装 MAVROS

```bash
# 安装 MAVROS 和相关工具
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras

# 安装地理数据库 (必须)
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 1.2 验证 PX4 连接

**USB 连接**:
```bash
# 检查设备
ls /dev/ttyACM*

# 启动 MAVROS
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
```

**串口连接 (Jetson UART)**:
```bash
# 检查设备
ls /dev/ttyTHS*

# 启动 MAVROS
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS0:921600
```

### 1.3 验证 IMU 数据

```bash
# 检查 IMU 话题频率 (应 >= 200Hz)
rostopic hz /mavros/imu/data_raw

# 查看 IMU 数据
rostopic echo /mavros/imu/data_raw --noarr
```

### 1.4 验证飞控连接状态

```bash
# 检查连接状态
rostopic echo /mavros/state

# 期望输出:
# connected: True
# armed: False
# mode: "MANUAL" 或 "STABILIZED"
```

### 检查清单

```
□ MAVROS 安装与配置
  ✅ 安装 mavros 和 mavros-extras
  ✅ 安装 geographiclib 数据集
  ✅ 确认 PX4 连接方式 (USB/串口) --> 当前为 USB
  ✅ roslaunch mavros px4.launch
  ✅ /mavros/state 显示 connected: True
  ✅ /mavros/imu/data_raw 频率 >= 200Hz
```

---

## 第二阶段: VINS-Fusion 配置与验证

### 2.1 配置文件位置

> **说明**: 配置目录名为 `realsense_d435i` 是 VINS-Fusion 官方提供的模板，用于 D455f 时**无需重新下载驱动**。
> realsense-ros 和 librealsense 是**通用驱动**，支持所有 RealSense 相机（D435i、D455、D455f 等）。
> 可以保持目录名不变直接使用，只需**重新标定相机内外参**即可。

所有配置文件集中存放于 `~/Projects/configs/`，由 Git 版本管理:

| 文件 | 源文件 (configs/) | 部署目标 |
|------|-------------------|----------|
| VINS 主配置 | `configs/vins/realsense_d435i/realsense_stereo_imu_config.yaml` | `catkin_ws/src/VINS-Fusion/config/realsense_d435i/` |
| 左相机标定 | `configs/vins/realsense_d435i/left.yaml` | 同上 |
| 右相机标定 | `configs/vins/realsense_d435i/right.yaml` | 同上 |
| 相机 Launch | `configs/realsense/yopo_d455f_camera.launch` | 可通过绝对路径直接启动，无需复制 |
| PX4 Launch | `configs/mavros/px4_yopo.launch` | 可通过绝对路径直接启动，无需复制 |

### 2.1.1 部署配置文件

每次修改 `configs/` 中的文件后，需同步到 VINS-Fusion 工作目录:

```bash
# 将 VINS 配置部署到 VINS-Fusion 工作目录
cp -r ~/Projects/configs/vins/realsense_d435i/* \
      ~/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/
```

### 2.2 配置文件检查

确认 `realsense_stereo_imu_config.yaml` 中的话题设置:

```yaml
imu_topic: "/mavros/imu/data_raw"             # PX4 IMU (必须)
image0_topic: "/camera/infra1/image_rect_raw"  # 左红外 (必须)
image1_topic: "/camera/infra2/image_rect_raw"  # 右红外 (必须)
```

### 2.3 PX4 IMU 外参标定

由于使用 PX4 IMU，需要重新标定 IMU 到相机的外参:

**方法 1 - 物理测量 (快速)**:
```yaml
# body_T_cam0 是 PX4 IMU 坐标系到左相机坐标系的变换
# 需要测量:
# 1. PX4 IMU 到相机的物理位置 (平移部分)
# 2. PX4 IMU 与相机的安装方向 (旋转部分)

# 设置 estimate_extrinsic: 1 让 VINS-Fusion 在线优化
estimate_extrinsic: 1
```

**方法 2 - Kalibr 标定 (精确)**:
```bash
# 使用 kalibr 工具进行 IMU-相机联合标定
# 参考: https://github.com/ethz-asl/kalibr
```

### 2.4 时间偏移参数

PX4 IMU 与相机无硬件时间同步，需要在线估计时间偏移:

```yaml
estimate_td: 1      # 启用时间偏移估计
td: 0.0             # 初始值 (可尝试 -0.02 ~ 0.02)
```

### 2.5 启动 VINS-Fusion 测试

**终端 1 - 启动 roscore**:
```bash
roscore
```

**终端 2 - 启动 MAVROS (PX4 IMU)**:
```bash
roslaunch ~/Projects/configs/mavros/px4_yopo.launch fcu_url:=/dev/ttyACM0:921600

# IMU 频率会自动设置为 200Hz，无需手动配置
# 详见: docs/MAVROS_IMU频率自动配置说明.md

# 检查 IMU 话题频率 (应显示 ~200Hz)
rostopic hz /mavros/imu/data_raw
```

**终端 3 - 启动 RealSense (YOPO 定制版 Launch)**:

```bash
roslaunch ~/Projects/configs/realsense/yopo_d455f_camera.launch
```

**终端 4 - 启动 VINS-Fusion**:
```bash
roslaunch vins vins_rviz.launch
rosrun vins vins_node ~/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

### 2.6 验证里程计输出

```bash
# 检查里程计话题频率 (应 >= 100Hz)
rostopic hz /vins_estimator/imu_propagate

# 查看里程计数据
rostopic echo /vins_estimator/imu_propagate --noarr
```

### 检查清单

```
□ VINS-Fusion 配置验证
  ✅ 确认 imu_topic 为 /mavros/imu/data_raw (PX4 IMU)
  ✅ 确认 image0/image1 为红外图像话题
  ✅ 确认 estimate_extrinsic: 1 (在线优化外参)
  ✅ 确认 estimate_td: 1 (在线估计时间偏移)
  ✅ 修改 output_path 为本地路径
  ✅ 部署配置文件到 VINS-Fusion 目录
  ✅ 按顺序启动 MAVROS → RealSense → VINS-Fusion
  ✅ 验证 /vins_estimator/imu_propagate 频率 >= 100Hz
  □ 移动相机测试位姿输出
```

---

## 第三阶段: RealSense YOPO 专用配置

### 3.1 深度图配置差异

VINS 和 YOPO 对 RealSense 的配置需求不同:

| 参数 | VINS 配置 | 当前相机配置 | YOPO 期望输入 |
|------|-----------|--------------|---------------|
| depth_width | 640 | **640** | 480 |
| depth_height | 480 | **480** | 270 |
| enable_color | true | **false** | false |
| enable_gyro/accel | true | **false** | false |

> ⚠️ **重要**: 当前相机输出 640x480 深度图，但 YOPO 神经网络期望 480x270 输入。
> **需要在 YOPO 规划器代码中添加 resize 逻辑** (从 640x480 → 480x270)。

### 3.2 YOPO 专用 Launch 文件

YOPO 定制版 Launch 文件已存在于 `configs/realsense/yopo_d455f_camera.launch`，预设了所有 YOPO 所需参数:

| 预设参数 | 值 | 说明 |
|----------|-----|------|
| enable_infra1/2 | true | VINS-Fusion 双目红外输入 |
| infra_width/height | 640×480 | VINS-Fusion 配置匹配 |
| enable_gyro/accel | **false** | 禁用，改用 PX4 IMU |
| depth_width/height | **640×480** | D455 官方推荐分辨率 (需 resize 到 480×270) |
| enable_color | false | 不需要 RGB，节省带宽 |
| initial_reset | true | 启动时重置相机，解决 USB 通信问题 |

> ⚠️ **分辨率说明**: D455 不支持 480×270 原生输出，当前使用 640×480。
> YOPO 规划器需要在代码中将深度图从 640×480 resize 到 480×270。

**启动方式**:
```bash
# 方式 1 - 通过绝对路径启动 (推荐，无需复制)
roslaunch ~/Projects/configs/realsense/yopo_d455f_camera.launch

# 方式 2 - 复制到 ROS 包后通过包名启动
cp ~/Projects/configs/realsense/yopo_d455f_camera.launch \
   ~/Projects/catkin_ws/src/realsense-ros/realsense2_camera/launch/
roslaunch realsense2_camera yopo_d455f_camera.launch
```

### 3.3 验证深度图参数

```bash
# 检查深度图分辨率
rostopic echo /camera/depth/image_rect_raw --noarr | head -20

# 应显示:
# width: 640
# height: 480
```

### 3.4 YOPO 规划器深度图 Resize (待实现)

由于 D455 不支持 480×270 原生分辨率，需要在 YOPO 规划器中添加 resize 逻辑:

```python
# 在 test_yopo_ros.py 的深度图回调中添加:
import cv2

def depth_callback(self, msg):
    # 原始深度图 640x480
    depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    
    # Resize 到 YOPO 期望的 480x270
    depth_resized = cv2.resize(depth_img, (480, 270), interpolation=cv2.INTER_NEAREST)
    
    # 使用 resized 深度图
    self.depth_image = depth_resized
```

> **注意**: 深度图 resize 应使用 `INTER_NEAREST` 插值，避免深度值失真。

### 检查清单

```
□ RealSense YOPO 配置
  □ 部署配置文件: cp configs → VINS-Fusion 目录
  □ 确认 YOPO 专用 launch 文件可启动
  □ 确认 D455f IMU 已禁用 (enable_gyro/accel: false)
  □ 深度图分辨率 640x480 (相机输出)
  □ YOPO 规划器 resize 深度图到 480x270 (待实现)
  □ 关闭 color 流节省带宽
```

---

## 第四阶段: SO3 控制器配置

### 4.1 Launch 文件位置

```
~/Projects/YOPO/Controller/src/so3_control/launch/controller_network.launch
```

### 4.2 关键参数

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

### 4.3 hover_thrust 标定方法

1. 在 QGroundControl 或手动模式下悬停飞机
2. 记录此时的油门值 (0-1 范围)
3. 该值即为 hover_thrust

**替代方法**: 根据飞机总重量和电机参数计算:
```
hover_thrust ≈ (总重量 × 9.8) / (最大推力)
```

### 4.4 验证控制器

```bash
# 启动控制器
roslaunch so3_control controller_network.launch

# 检查输出话题
rostopic echo /mavros/setpoint_raw/attitude --noarr
```

### 检查清单

```
□ SO3 控制器配置
  □ 确认 is_simulation 为 false
  □ 标定 hover_thrust 参数
  □ 确认 odom 重映射到 /vins_estimator/imu_propagate
  □ 确认 imu 重映射到 /mavros/imu/data_raw
  □ 启动控制器验证无报错
```

---

## 第五阶段: YOPO 规划器测试

### 5.1 配置检查

确认 `test_yopo_ros.py` 中的 settings:

```python
settings = {
    'odom_topic': '/vins_estimator/imu_propagate',   # 里程计
    'depth_topic': '/camera/depth/image_rect_raw',   # 深度图
    'ctrl_topic': '/so3_control/pos_cmd',            # 控制指令
    'use_tensorrt': 1,                               # 启用 TensorRT
    'plan_from_reference': True,
    'visualize': False,                              # 实飞时关闭
}
```

### 5.2 启动 YOPO (使用 TensorRT)

```bash
conda activate yopo
cd ~/Projects/YOPO/YOPO

# 使用 TensorRT 模型
python test_yopo_ros.py --trial=1 --epoch=50 --weight=saved/YOPO_1/yopo_trt.pth
```

### 5.3 验证输出

```bash
# 检查控制指令话题
rostopic hz /so3_control/pos_cmd  # 应为 50Hz

# 查看规划输出
rostopic echo /so3_control/pos_cmd --noarr
```

### 检查清单

```
□ YOPO 规划器测试
  □ 确认使用 TensorRT 模型
  □ 确认 odom_topic 正确
  □ 确认 depth_topic 正确
  □ visualize 设为 False
  □ 启动规划器无报错
  □ /so3_control/pos_cmd 正常发布
```

---

## 第六阶段: 系统集成测试

### 6.1 分步启动顺序

> **重要**: 使用 PX4 IMU 方案时，**必须先启动 MAVROS**，再启动其他节点。

按以下顺序在不同终端启动:

**终端 1 - roscore**:
```bash
roscore
```

**终端 2 - MAVROS (提供 PX4 IMU)**:
```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
```

**终端 3 - RealSense (YOPO 定制版 Launch)**:
```bash
roslaunch ~/Projects/configs/realsense/yopo_d455f_camera.launch
```

**终端 4 - VINS-Fusion**:
```bash
rosrun vins vins_node ~/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

**终端 5 - SO3 控制器**:
```bash
source ~/Projects/YOPO/Controller/devel/setup.bash
roslaunch so3_control controller_network.launch
```

**终端 6 - YOPO 规划器**:
```bash
conda activate yopo
cd ~/Projects/YOPO/YOPO
python test_yopo_ros.py --trial=1 --epoch=50
```

### 6.2 话题连通性检查

```bash
# 一键检查所有关键话题
rostopic list | grep -E "(camera|vins|mavros|so3_control)"

# 期望看到:
# /camera/depth/image_rect_raw
# /camera/infra1/image_rect_raw
# /camera/infra2/image_rect_raw
# /mavros/imu/data_raw       <-- PX4 IMU
# /mavros/state
# /vins_estimator/imu_propagate
# /mavros/setpoint_raw/attitude
# /so3_control/pos_cmd
```

> **注意**: 使用 PX4 IMU 方案时，不会有 `/camera/imu` 话题。

### 6.3 话题频率检查

```bash
# PX4 IMU (>=200Hz)
rostopic hz /mavros/imu/data_raw

# 深度图 (30Hz)
rostopic hz /camera/depth/image_rect_raw

# 里程计 (>=100Hz)
rostopic hz /vins_estimator/imu_propagate

# 控制指令 (50Hz)
rostopic hz /so3_control/pos_cmd
```

### 6.4 端到端延迟测试

使用 rqt_graph 可视化数据流:
```bash
rqt_graph
```

### 检查清单

```
□ 系统集成测试
  □ 按顺序启动所有节点 (MAVROS 必须先启动)
  □ 所有关键话题存在
  □ /mavros/imu/data_raw >= 200Hz
  □ /camera/depth/image_rect_raw 30Hz
  □ /vins_estimator/imu_propagate >= 100Hz
  □ /so3_control/pos_cmd 50Hz
  □ 无节点报错或警告
```

---

## 第七阶段: 实飞前检查清单

### 7.1 硬件检查

```
□ 硬件检查
  □ 电池电压充足
  □ 螺旋桨安装牢固
  □ D455f 相机固定稳固
  □ Jetson 供电稳定
  □ 飞控与 Jetson 连接正常 (USB/串口)
```

### 7.2 软件检查

```
□ 软件检查
  □ 所有节点启动无报错
  □ MAVROS connected: True
  □ PX4 IMU 数据正常 (/mavros/imu/data_raw)
  □ VINS-Fusion 里程计稳定 (无跳变)
  □ YOPO 规划器正常输出
  □ hover_thrust 已正确标定
```

### 7.3 安全检查

```
□ 安全检查
  □ 遥控器在手，可随时切回手动
  □ 测试区域无障碍物干扰
  □ 紧急停止开关就绪
  □ visualize 已关闭 (减少延迟)
```

### 7.4 约束条件确认

```
□ 约束条件确认 (必须)
  □ VINS 使用 PX4 IMU (/mavros/imu/data_raw)
  □ VINS 使用红外图像 (非 RGB)
  □ D455f IMU 已禁用
  □ 深度图相机输出 640x480，YOPO resize 到 480x270
  □ 坐标系为 NWU
  □ visualize = False
```

---

## 故障排查速查

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| VIO 发散/跳变 | IMU 话题错误 | 确认使用 `/mavros/imu/data_raw` |
| VIO 不稳定 | 时间偏移估计不准 | 调整 `td` 初值 (-0.02 ~ 0.02) |
| VIO 外参错误 | PX4 IMU 外参未标定 | 使用 kalibr 标定或设置 `estimate_extrinsic: 1` |
| 无 IMU 数据 | MAVROS 未启动 | 先启动 MAVROS |
| 规划器无输出 | 缺少深度图/里程计 | 检查话题订阅 |
| TensorRT 加载失败 | 模型未转换 | 执行 yopo_trt_transfer.py |
| 控制器无响应 | 话题重映射错误 | 检查 launch 文件 |
| MAVROS 无连接 | 串口配置错误 | 检查 fcu_url 参数 |
| numpy 版本错误 | 未激活 yopo 环境 | conda activate yopo |

---

## 相关文档

| 文档 | 路径 |
|------|------|
| 硬件通信节点说明 | `~/Projects/docs/YOPO_硬件通信节点说明.md` |
| 虚拟环境配置说明 | `~/Projects/docs/YOPO_虚拟环境与隔离配置说明.md` |
| **MAVROS IMU 频率自动配置** | `~/Projects/docs/MAVROS_IMU频率自动配置说明.md` |
| 第三方依赖说明 | `~/Projects/third_party.md` |

---

## 版本历史

| 日期 | 变更 |
|------|------|
| 2026-02-04 | 初始创建，基于当前部署状态生成 |
| 2026-02-05 | 完成 TensorRT 模型转换 |
| 2026-02-06 | 统一配置文件路径至 configs/，更新所有命令为可直接执行 |
| 2026-02-06 | **切换 IMU 方案**: 从 D455f IMU 改为 PX4 IMU |
| 2026-02-06 | **深度图分辨率调整**: 640x480 (D455 官方推荐)，需在 YOPO 中 resize 到 480x270 |
| 2026-02-07 | **IMU 频率自动配置**: 创建 yopo_tools 包，启动时自动设置 200Hz |