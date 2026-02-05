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
| catkin_ws 编译 | ✅ 已完成 | /home/amov/Projects/catkin_ws |
| yopo conda 环境 | ✅ 已创建 | Python 3.8, Jetson PyTorch |
| conda hooks 隔离 | ✅ 已配置 | PYTHONPATH 优先级正确 |
| Controller 编译 | ✅ 已完成 | /home/amov/Projects/YOPO/Controller |
| rosfix OpenCV 修复 | ✅ 已配置 | ~/.bashrc |
| 训练模型 | ✅ 已存在 | saved/YOPO_1/epoch50.pth |
| TensorRT 模型转换 | ✅ 已完成 | yopo_trt.pth |

### 待完成

| 项目 | 优先级 | 预计耗时 |
|------|--------|----------|
| VINS-Fusion 配置验证 | 高 | 10 分钟 |
| MAVROS 配置 | 中 | 15 分钟 |
| SO3 控制器参数标定 | 中 | 30 分钟 |
| 系统集成测试 | 高 | 20 分钟 |

---


## 第一阶段: VINS-Fusion 配置与验证

### 1.1 配置文件位置

> **说明**: 配置目录名为 `realsense_d435i` 是 VINS-Fusion 官方提供的模板，用于 D455f 时**无需重新下载驱动**。
> realsense-ros 和 librealsense 是**通用驱动**，支持所有 RealSense 相机（D435i、D455、D455f 等）。
> 可以保持目录名不变直接使用，只需**重新标定相机内外参**即可。

| 文件 | 路径 |
|------|------|
| 主配置 | `/home/amov/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml` |
| 左相机标定 | `/home/amov/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/left.yaml` |
| 右相机标定 | `/home/amov/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/right.yaml` |
| 相机 Launch | `/home/amov/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/rs_camera.launch` |

### 1.2 配置文件检查

确认 `realsense_stereo_imu_config.yaml` 中的话题设置:

```yaml
imu_topic: "/camera/imu"                    # D455f IMU (必须)
image0_topic: "/camera/infra1/image_rect_raw"  # 左红外 (必须)
image1_topic: "/camera/infra2/image_rect_raw"  # 右红外 (必须)
```

### 1.3 可能需要修改的参数

根据实际 D455f 重新标定内外参:

```yaml
# IMU 噪声参数 (根据实际情况调整)
acc_n: 0.1          # 加速度计噪声
gyr_n: 0.01         # 陀螺仪噪声
acc_w: 0.001        # 加速度计偏置随机游走
gyr_w: 0.0001       # 陀螺仪偏置随机游走

# 输出路径 (修改为本地路径)
output_path: "/home/amov/output/"
pose_graph_save_path: "/home/amov/output/pose_graph/"
```

### 1.4 启动 VINS-Fusion 测试

**终端 1 - 启动 RealSense (VINS 专用配置)**:
```bash
roslaunch vins config/realsense_d435i/rs_camera.launch
```

**终端 2 - 启动 VINS-Fusion**:
```bash
roslaunch vins vins_rviz.launch
rosrun vins vins_node /home/amov/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

### 1.5 验证里程计输出

```bash
# 检查里程计话题频率 (应 >= 100Hz)
rostopic hz /vins_estimator/imu_propagate

# 查看里程计数据
rostopic echo /vins_estimator/imu_propagate --noarr
```

### 检查清单

```
□ VINS-Fusion 配置验证
  □ 确认 imu_topic 为 /camera/imu (D455f IMU)
  □ 确认 image0/image1 为红外图像话题
  □ 修改 output_path 为本地路径
  □ 启动 VINS-Fusion
  □ 验证 /vins_estimator/imu_propagate 频率 >= 100Hz
  □ 移动相机测试位姿输出
```

---

## 第二阶段: RealSense YOPO 专用配置

### 2.1 深度图配置差异

VINS 和 YOPO 对 RealSense 的配置需求不同:

| 参数 | VINS 配置 | YOPO 配置 |
|------|-----------|-----------|
| depth_width | 640 | **480** |
| depth_height | 480 | **270** |
| enable_color | true | **false** |

### 2.2 创建 YOPO 专用 Launch 文件

建议创建 `/home/amov/Projects/YOPO/launch/yopo_realsense.launch`:

```xml
<launch>
  <!-- RealSense D455f for YOPO -->
  <arg name="depth_width"         default="480"/>
  <arg name="depth_height"        default="270"/>
  <arg name="depth_fps"           default="30"/>
  
  <arg name="infra1_width"        default="640"/>
  <arg name="infra1_height"       default="480"/>
  <arg name="infra1_fps"          default="30"/>
  
  <arg name="infra2_width"        default="640"/>
  <arg name="infra2_height"       default="480"/>
  <arg name="infra2_fps"          default="30"/>
  
  <arg name="gyro_fps"            default="200"/>
  <arg name="accel_fps"           default="200"/>
  
  <arg name="enable_color"        default="false"/>
  <arg name="enable_depth"        default="true"/>
  <arg name="enable_infra1"       default="true"/>
  <arg name="enable_infra2"       default="true"/>
  <arg name="enable_imu"          default="true"/>
  <arg name="unite_imu_method"    default="linear_interpolation"/>
  
  <include file="$(find realsense2_camera)/launch/rs_camera.launch">
    <arg name="depth_width"       value="$(arg depth_width)"/>
    <arg name="depth_height"      value="$(arg depth_height)"/>
    <arg name="depth_fps"         value="$(arg depth_fps)"/>
    <arg name="infra_width"       value="$(arg infra1_width)"/>
    <arg name="infra_height"      value="$(arg infra1_height)"/>
    <arg name="infra_fps"         value="$(arg infra1_fps)"/>
    <arg name="gyro_fps"          value="$(arg gyro_fps)"/>
    <arg name="accel_fps"         value="$(arg accel_fps)"/>
    <arg name="enable_color"      value="$(arg enable_color)"/>
    <arg name="enable_depth"      value="$(arg enable_depth)"/>
    <arg name="enable_infra1"     value="$(arg enable_infra1)"/>
    <arg name="enable_infra2"     value="$(arg enable_infra2)"/>
    <arg name="enable_gyro"       value="$(arg enable_imu)"/>
    <arg name="enable_accel"      value="$(arg enable_imu)"/>
    <arg name="unite_imu_method"  value="$(arg unite_imu_method)"/>
  </include>
</launch>
```

### 2.3 验证深度图参数

```bash
# 检查深度图分辨率
rostopic echo /camera/depth/image_rect_raw --noarr | head -20

# 应显示:
# width: 480
# height: 270
```

### 检查清单

```
□ RealSense YOPO 配置
  □ 创建 YOPO 专用 launch 文件 (可选)
  □ 深度图分辨率 480x270
  □ IMU 频率 200Hz
  □ 关闭 color 流节省带宽
```

---

## 第三阶段: MAVROS 配置

### 3.1 安装 MAVROS (如未安装)

```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 3.2 PX4 连接配置

**USB 连接**:
```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
```

**串口连接**:
```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS0:921600
```

### 3.3 验证连接

```bash
# 检查连接状态
rostopic echo /mavros/state

# 期望输出:
# connected: True
# armed: False
# mode: "MANUAL" 或 "STABILIZED"
```

### 3.4 设置 OFFBOARD 模式

```bash
# 设置模式
rosservice call /mavros/set_mode "custom_mode: 'OFFBOARD'"

# 解锁
rosservice call /mavros/cmd/arming "value: true"
```

### 检查清单

```
□ MAVROS 配置
  □ 安装 mavros 和 mavros-extras
  □ 确认 PX4 连接方式 (USB/串口)
  □ roslaunch mavros px4.launch
  □ /mavros/state 显示 connected: True
  □ 测试 OFFBOARD 模式切换
```

---

## 第四阶段: SO3 控制器配置

### 4.1 Launch 文件位置

```
/home/amov/Projects/YOPO/Controller/src/so3_control/launch/controller_network.launch
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
cd /home/amov/Projects/YOPO/YOPO

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

按以下顺序在不同终端启动:

**终端 1 - roscore**:
```bash
roscore
```

**终端 2 - RealSense**:
```bash
roslaunch realsense2_camera rs_camera.launch \
    depth_width:=480 depth_height:=270 \
    enable_infra1:=true enable_infra2:=true \
    enable_gyro:=true enable_accel:=true \
    unite_imu_method:=linear_interpolation
```

**终端 3 - VINS-Fusion**:
```bash
rosrun vins vins_node /home/amov/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

**终端 4 - MAVROS**:
```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
```

**终端 5 - SO3 控制器**:
```bash
source /home/amov/Projects/YOPO/Controller/devel/setup.bash
roslaunch so3_control controller_network.launch
```

**终端 6 - YOPO 规划器**:
```bash
conda activate yopo
cd /home/amov/Projects/YOPO/YOPO
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
# /camera/imu
# /vins_estimator/imu_propagate
# /mavros/state
# /mavros/imu/data_raw
# /mavros/setpoint_raw/attitude
# /so3_control/pos_cmd
```

### 6.3 话题频率检查

```bash
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
  □ 按顺序启动所有节点
  □ 所有关键话题存在
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
  □ 飞控与 Jetson 连接正常
```

### 7.2 软件检查

```
□ 软件检查
  □ 所有节点启动无报错
  □ VINS-Fusion 里程计稳定 (无跳变)
  □ MAVROS connected: True
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
  □ VINS 使用 D455f IMU (非 PX4 IMU)
  □ VINS 使用红外图像 (非 RGB)
  □ 深度图 480x270 (16:9)
  □ 坐标系为 NWU
  □ visualize = False
```

---

## 故障排查速查

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| VIO 发散/跳变 | IMU 源错误 | 确认使用 `/camera/imu` |
| 规划器无输出 | 缺少深度图/里程计 | 检查话题订阅 |
| TensorRT 加载失败 | 模型未转换 | 执行 yopo_trt_transfer.py |
| 控制器无响应 | 话题重映射错误 | 检查 launch 文件 |
| MAVROS 无连接 | 串口配置错误 | 检查 fcu_url 参数 |
| numpy 版本错误 | 未激活 yopo 环境 | conda activate yopo |

---

## 相关文档

| 文档 | 路径 |
|------|------|
| 硬件通信节点说明 | `/home/amov/Projects/YOPO_硬件通信节点说明.md` |
| 虚拟环境配置说明 | `/home/amov/Projects/YOPO_虚拟环境与隔离配置说明.md` |
| 驱动版本说明 | `/home/amov/Projects/YOPO_驱动版本说明.md` |
| 部署 Skill | `/home/amov/Projects/.cursor/skills/yopo-deployment/SKILL.md` |

---

## 版本历史

| 日期 | 变更 |
|------|------|
| 2026-02-04 | 初始创建，基于当前部署状态生成 |
| 2026-02-05 | 完成 TensorRT 模型转换 |
