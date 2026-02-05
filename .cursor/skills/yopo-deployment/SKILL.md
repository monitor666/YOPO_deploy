---
name: yopo-deployment
description: 指导 YOPO 无人机自主导航系统在 Jetson 平台的实机部署。当用户要求部署 YOPO、配置 VINS-Fusion、调试硬件通信或进行系统集成测试时使用此 skill。
---

# YOPO 实机部署指南

YOPO (You Only Plan Once) 是基于神经网络的无人机自主导航规划系统。

## 当前部署状态

> 更新日期: 2026-02-05

| 阶段 | 状态 | 说明 |
|------|------|------|
| 驱动安装 | ✅ 完成 | librealsense 2.50.0, realsense-ros, VINS-Fusion |
| conda 环境 | ✅ 完成 | yopo 环境 + ROS 隔离配置 |
| Controller 编译 | ✅ 完成 | SO3 控制器已编译 |
| TensorRT 转换 | ✅ 完成 | yopo_trt.pth |
| VINS-Fusion 验证 | ⏳ 待完成 | 需验证 D455f 配置 |
| MAVROS 配置 | ⏳ 待完成 | 需连接 PX4 飞控 |
| 系统集成测试 | ⏳ 待完成 | 需全链路验证 |

---

## 强制约束

| ID | 约束内容 | 原因 |
|----|----------|------|
| C-001 | VINS-Fusion **必须**使用 D455f IMU | D455f 内部已完成 IMU-相机硬件时间同步 |
| C-002 | VINS-Fusion **必须**使用双目红外图像 | 红外不受光照影响，纹理更稳定 |
| C-003 | 里程计**必须**使用 NWU 坐标系 | YOPO 训练数据使用 NWU |
| C-004 | 深度图**必须** 480×270 (16:9) | 神经网络输入维度固定 |
| E-004 | **禁止**从 PyPI 安装 torch | 必须使用 Jetson 专用版本 |

### 禁止操作

| 操作 | 原因 |
|------|------|
| ❌ 使用 PX4 IMU 作为 VINS 输入 | 时间同步问题导致 VIO 发散 |
| ❌ 使用 RGB 图像作为 VINS 输入 | 光照敏感，不如红外稳定 |
| ❌ 实飞时 visualize=True | 增加计算负担导致延迟 |

---

## 关键路径

| 资源 | 路径 |
|------|------|
| catkin_ws | `/home/amov/Projects/catkin_ws` |
| YOPO 项目 | `/home/amov/Projects/YOPO` |
| Controller | `/home/amov/Projects/YOPO/Controller` |
| TensorRT 模型 | `/home/amov/Projects/YOPO/YOPO/saved/YOPO_1/yopo_trt.pth` |
| 训练模型 | `/home/amov/Projects/YOPO/YOPO/saved/YOPO_1/epoch50.pth` |
| VINS 配置 | `/home/amov/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/` |
| OpenCV 补丁 | `/home/amov/Projects/patches/opencv4_compat.h` |
| conda hooks 源 | `/home/amov/Projects/configs/conda/hooks/` |

---

## 待完成任务

### 1. VINS-Fusion 配置验证

**配置文件**: `/home/amov/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml`

确认话题设置：
```yaml
imu_topic: "/camera/imu"                      # D455f IMU (必须)
image0_topic: "/camera/infra1/image_rect_raw" # 左红外 (必须)
image1_topic: "/camera/infra2/image_rect_raw" # 右红外 (必须)
output_path: "/home/amov/output/"             # 修改为本地路径
```

**验证步骤**:
```bash
# 终端 1 - 启动相机
roslaunch realsense2_camera rs_camera.launch \
    enable_infra1:=true enable_infra2:=true \
    enable_gyro:=true enable_accel:=true \
    unite_imu_method:=linear_interpolation

# 终端 2 - 启动 VINS
rosrun vins vins_node /home/amov/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml

# 终端 3 - 验证输出
rostopic hz /vins_estimator/imu_propagate  # 应 >= 100Hz
```

### 2. MAVROS 配置

```bash
# 安装 (如未安装)
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

# 启动 (USB 连接)
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600

# 验证
rostopic echo /mavros/state  # connected: True
```

### 3. SO3 控制器参数

**Launch 文件**: `/home/amov/Projects/YOPO/Controller/src/so3_control/launch/controller_network.launch`

```xml
<param name="is_simulation" value="false"/>
<param name="hover_thrust" value="0.38"/>  <!-- 需实际标定 -->
<remap from="~odom" to="/vins_estimator/imu_propagate"/>
<remap from="~imu" to="/mavros/imu/data_raw"/>
```

**hover_thrust 标定**: 悬停时记录油门值 (0-1 范围)

---

## 系统启动流程

按顺序在不同终端启动：

```bash
# 1. roscore
roscore

# 2. RealSense (YOPO 深度图配置)
roslaunch realsense2_camera rs_camera.launch \
    depth_width:=480 depth_height:=270 \
    enable_infra1:=true enable_infra2:=true \
    enable_gyro:=true enable_accel:=true \
    unite_imu_method:=linear_interpolation

# 3. VINS-Fusion
rosrun vins vins_node /home/amov/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml

# 4. MAVROS
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600

# 5. SO3 控制器
source /home/amov/Projects/YOPO/Controller/devel/setup.bash
roslaunch so3_control controller_network.launch

# 6. YOPO 规划器
conda activate yopo
cd /home/amov/Projects/YOPO/YOPO
python test_yopo_ros.py --trial=1 --epoch=50
```

---

## 话题通信矩阵

| 发布节点 | 话题 | 频率 | 订阅节点 |
|----------|------|------|----------|
| D455f | `/camera/depth/image_rect_raw` | 30Hz | YOPO 规划器 |
| D455f | `/camera/infra1/image_rect_raw` | 30Hz | VINS-Fusion |
| D455f | `/camera/infra2/image_rect_raw` | 30Hz | VINS-Fusion |
| D455f | `/camera/imu` | 200Hz | VINS-Fusion |
| VINS-Fusion | `/vins_estimator/imu_propagate` | 100Hz+ | YOPO, SO3 控制器 |
| YOPO | `/so3_control/pos_cmd` | 50Hz | SO3 控制器 |
| SO3 控制器 | `/mavros/setpoint_raw/attitude` | 50Hz | MAVROS |

---

## 故障排查

| 问题 | 症状 | 解决方案 |
|------|------|----------|
| numpy 版本错误 | 显示 1.17.x | `conda deactivate && conda activate yopo` |
| rospy 无法导入 | ModuleNotFoundError | 检查 conda hooks 脚本是否存在 |
| roslaunch 报 OpenCV 错误 | undefined symbol | 确认 ~/.bashrc 有 rosfix 函数和别名 |
| VIO 发散 | 位置跳变 > 1m | 确认使用 D455f IMU 而非 PX4 IMU |
| TensorRT 加载失败 | 模型未找到 | 确认 yopo_trt.pth 存在 |

### 验证 conda 环境

```bash
conda activate yopo
python -c "import numpy; print(numpy.__version__, numpy.__file__)"
# 期望: 1.24.4 /home/amov/miniforge3/envs/yopo/...
```

---

## 参考文档

| 文档 | 说明 |
|------|------|
| [driver-installation.md](driver-installation.md) | 驱动版本信息 (已完成) |
| [environment-setup.md](environment-setup.md) | conda 环境配置 (已完成) |
| [hardware-integration.md](hardware-integration.md) | 硬件通信详情 |
| `/home/amov/Projects/docs/YOPO_部署工作清单.md` | 完整部署清单 |
| `/home/amov/Projects/third_party.md` | 第三方依赖克隆命令 |
