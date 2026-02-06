---
name: yopo-deployment
description: 指导 YOPO 无人机自主导航系统在 Jetson 平台的实机部署。当用户要求部署 YOPO、配置 VINS-Fusion、调试硬件通信或进行系统集成测试时使用此 skill。
---

# YOPO 实机部署指南

YOPO (You Only Plan Once) 是基于神经网络的无人机自主导航规划系统。

---

## 文件存放规范 (必须遵守)

> **核心原则**: `~/Projects/` 是项目根目录，所有由 AI 生成或修改的配置文件、补丁文件、脚本文件**必须**存放在以下对应目录中，**禁止**散落在项目根目录或其他任意位置。

### 目录结构

```
~/Projects/
├── configs/          # 配置文件 (YAML, launch, JSON, INI 等)
│   ├── bash/         # Shell 环境配置 (如 rosfix.sh)
│   ├── conda/        # conda 环境相关配置
│   │   └── hooks/    # conda activate/deactivate hooks
│   ├── realsense/    # RealSense 相机 launch 文件
│   └── vins/         # VINS-Fusion 配置
│       └── realsense_d435i/   # 双目红外+IMU 配置组
├── patches/          # 补丁文件 (.h, .patch, .diff 等)
├── scripts/          # 脚本文件 (.sh, .py 等部署/运维脚本)
├── docs/             # 文档文件 (.md)
├── catkin_ws/        # ROS catkin 工作空间 (编译产物)
└── YOPO/             # YOPO 项目源码 (上游仓库)
```

### 各目录详细规范

#### `configs/` — 配置文件

| 规则 | 说明 |
|------|------|
| 存放内容 | 所有 ROS launch 文件、YAML 配置、conda hooks、bash 环境脚本等 |
| 命名约定 | 按功能模块分子目录: `realsense/`, `vins/`, `conda/`, `bash/` 等 |
| 使用方法 | 部分文件需要复制到 ROS 工作空间才能生效，**每个文件开头必须注释说明使用方法** |
| 注释要求 | 文件开头必须包含: (1) 文件用途说明 (2) 部署/复制命令 (3) 启动命令示例 |

**示例 — 配置文件开头注释模板**:
```yaml
# ============================================================================
# [文件用途简述]
# ============================================================================
#
# 部署步骤:
#   cp ~/Projects/configs/xxx/this_file ~/Projects/catkin_ws/src/xxx/
#
# 使用方法:
#   roslaunch xxx this_file
#   或
#   rosrun xxx node ~/Projects/catkin_ws/src/xxx/this_file
#
# ============================================================================
```

#### `patches/` — 补丁文件

| 规则 | 说明 |
|------|------|
| 存放内容 | C/C++ 兼容性头文件、.patch/.diff 文件、代码修复片段 |
| 命名约定 | 清晰描述修复目标，如 `opencv4_compat.h` |
| 注释要求 | 文件开头说明: 修复什么问题、适用于哪个版本、如何应用 |

#### `scripts/` — 脚本文件

| 规则 | 说明 |
|------|------|
| 存放内容 | 部署脚本、运维脚本、一键启动脚本、数据处理脚本等 |
| 命名约定 | 使用动词前缀: `deploy_xxx.sh`, `start_xxx.sh`, `check_xxx.sh` |
| 注释要求 | 文件开头说明: 脚本功能、使用前提、使用方法 |
| 权限要求 | Shell 脚本需设置可执行权限 `chmod +x` |

**示例 — 脚本文件开头注释模板**:
```bash
#!/bin/bash
# ============================================================================
# [脚本功能简述]
# ============================================================================
#
# 前提条件:
#   - xxx 已安装
#   - yyy 已配置
#
# 使用方法:
#   bash ~/Projects/scripts/this_script.sh
#   或
#   ~/Projects/scripts/this_script.sh (需要 chmod +x)
#
# ============================================================================
```

### 生成文件时的强制流程

当 AI 助手生成或修改配置文件、补丁文件、脚本文件时，**必须**按照以下流程操作:

1. **确定文件类型** → 选择对应目录 (`configs/`, `patches/`, `scripts/`)
2. **确定子目录** → 按功能模块放入合适的子目录
3. **添加开头注释** → 包含用途、部署步骤、使用方法
4. **如果文件需要部署到其他位置** → 在注释中写明 `cp` 命令
5. **更新部署清单** → 如涉及新增文件，更新 `docs/YOPO_部署工作清单.md`

---

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
| catkin_ws | `~/Projects/catkin_ws` |
| YOPO 项目 | `~/Projects/YOPO` |
| Controller | `~/Projects/YOPO/Controller` |
| TensorRT 模型 | `~/Projects/YOPO/YOPO/saved/YOPO_1/yopo_trt.pth` |
| 训练模型 | `~/Projects/YOPO/YOPO/saved/YOPO_1/epoch50.pth` |
| VINS 配置 (源) | `~/Projects/configs/vins/realsense_d435i/` |
| VINS 配置 (部署) | `~/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/` |
| 相机 Launch | `~/Projects/configs/realsense/yopo_d455f_camera.launch` |
| OpenCV 补丁 | `~/Projects/patches/opencv4_compat.h` |
| conda hooks 源 | `~/Projects/configs/conda/hooks/` |
| 部署脚本 | `~/Projects/scripts/` |

---

## 待完成任务

### 1. VINS-Fusion 配置验证

**源文件**: `~/Projects/configs/vins/realsense_d435i/realsense_stereo_imu_config.yaml`
**部署位置**: `~/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml`

确认话题设置：
```yaml
imu_topic: "/camera/imu"                      # D455f IMU (必须)
image0_topic: "/camera/infra1/image_rect_raw" # 左红外 (必须)
image1_topic: "/camera/infra2/image_rect_raw" # 右红外 (必须)
output_path: "/home/amov/Projects/output/"             # 修改为本地路径
```

**验证步骤**:
```bash
# 终端 1 - 启动相机 (YOPO 定制版 Launch)
roslaunch ~/Projects/configs/realsense/yopo_d455f_camera.launch

# 终端 2 - 启动 VINS
rosrun vins vins_node ~/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml

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

**Launch 文件**: `~/Projects/YOPO/Controller/src/so3_control/launch/controller_network.launch`

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

# 2. RealSense (YOPO 定制版 Launch)
roslaunch ~/Projects/configs/realsense/yopo_d455f_camera.launch

# 3. VINS-Fusion
rosrun vins vins_node ~/Projects/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml

# 4. MAVROS
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600

# 5. SO3 控制器
source ~/Projects/YOPO/Controller/devel/setup.bash
roslaunch so3_control controller_network.launch

# 6. YOPO 规划器
conda activate yopo
cd ~/Projects/YOPO/YOPO
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
| `~/Projects/docs/YOPO_部署工作清单.md` | 完整部署清单 |
| `~/Projects/third_party.md` | 第三方依赖克隆命令 |
