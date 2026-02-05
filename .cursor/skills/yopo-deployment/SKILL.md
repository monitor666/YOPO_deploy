---
name: yopo-deployment
description: 指导 YOPO 无人机自主导航系统在 Jetson 平台的完整部署，包括驱动安装、环境配置、ROS 隔离和硬件集成。当用户要求部署 YOPO、配置 RealSense、设置 VINS-Fusion、处理 conda/ROS 环境冲突或调试 YOPO 相关问题时使用此 skill。
---

# YOPO 部署指南

YOPO (You Only Plan Once) 是基于神经网络的无人机自主导航规划系统，部署于 NVIDIA Jetson 平台。

## 快速部署检查清单

部署前复制此清单跟踪进度：

```
部署进度:
- [ ] 1. 驱动安装
  - [ ] librealsense 2.50.0 编译安装
  - [ ] realsense-ros (ros1-legacy 分支) 克隆
  - [ ] VINS-Fusion 克隆并修复 OpenCV 4.x 兼容性
  - [ ] catkin_ws 编译成功
- [ ] 2. conda 环境配置
  - [ ] yopo 环境创建 (Python 3.8)
  - [ ] Jetson 专用 PyTorch 安装
  - [ ] 依赖包安装完成
- [ ] 3. ROS 隔离配置
  - [ ] activate.d hook 脚本创建
  - [ ] deactivate.d hook 脚本创建
  - [ ] PYTHONPATH 优先级验证
- [ ] 4. 硬件集成验证
  - [ ] D435i 话题发布正常
  - [ ] VINS-Fusion 里程计输出正常
  - [ ] MAVROS 连接 PX4 正常
- [ ] 5. YOPO 运行测试
  - [ ] TensorRT 模型加载成功
  - [ ] 规划器正常输出轨迹
```

## 强制约束 (部署时必须遵守)

| ID | 约束内容 |
|----|----------|
| C-001 | VINS-Fusion 的 IMU 输入**必须**使用 D435i 自带 IMU，**禁止**使用 PX4 飞控 IMU |
| C-002 | VINS-Fusion 的图像输入**必须**使用 D435i 双目红外图像 |
| C-003 | 所有里程计数据**必须**使用 NWU (北-西-上) 坐标系 |
| C-004 | 深度图分辨率**必须**保持 16:9 宽高比 (480×270) |
| E-004 | **禁止**从 PyPI 安装 CUDA 版本的 torch/torchvision，必须使用 Jetson 专用版本 |

## 禁止操作

| 操作 | 原因 |
|------|------|
| ❌ 使用 PX4 IMU 作为 VINS 输入 | 时间同步问题导致 VIO 发散 |
| ❌ 使用 RGB 图像作为 VINS 输入 | 光照敏感，不如红外稳定 |
| ❌ 修改深度图为非 16:9 比例 | 神经网络输入维度不匹配 |
| ❌ 在 ENU 坐标系下运行 | 与训练数据坐标系不一致 |
| ❌ 实飞时开启 visualize=True | 增加计算负担导致延迟 |

---

## 阶段 1: 驱动安装

### 1.1 librealsense 编译安装

```bash
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense && git checkout v2.50.0

mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=true -DBUILD_WITH_CUDA=true -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install && sudo ldconfig
```

### 1.2 ROS 驱动克隆

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src

# realsense-ros
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros && git checkout ros1-legacy && cd ..

# VINS-Fusion
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
```

### 1.3 VINS-Fusion OpenCV 4.x 兼容性修复

需在多个文件开头添加兼容性宏，详见 [driver-installation.md](driver-installation.md)

### 1.4 OpenCV 版本冲突修复

在 `~/.bashrc` 添加：

```bash
rosfix() {
    LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.5" "$@"
}
alias roslaunch='rosfix roslaunch'
alias rosrun='rosfix rosrun'
alias rviz='rosfix rviz'
```

### 1.5 编译 catkin_ws

```bash
cd ~/catkin_ws && catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

---

## 阶段 2: conda 环境配置

### 2.1 创建 yopo 环境

```bash
conda create -n yopo python=3.8 -y
conda activate yopo
```

### 2.2 安装 Jetson 专用 PyTorch

**重要**: 使用 NVIDIA 预装的 Jetson 专用版本，禁止从 PyPI 安装。

```bash
# 从 NVIDIA 提供的 wheel 安装 (版本: torch 2.0.0a0+nv23.3)
# torchvision 0.15.1 配套版本
```

### 2.3 安装其他依赖

```bash
pip install numpy==1.24.4 scipy==1.10.1 scikit-learn==1.3.2
pip install opencv-python-headless==4.12.0.88 open3d==0.18.0 pillow==10.4.0
pip install ruamel.yaml==0.17.21 tensorboard==2.14.0 rich scikit-build==0.18.1 empy==4.2
pip install catkin_pkg rospkg netifaces torch2trt==0.5.0
```

---

## 阶段 3: ROS 隔离配置 (关键步骤)

### 问题背景

conda 和 ROS 存在 PYTHONPATH 冲突，导致导入错误版本的 numpy (ROS 1.17 vs conda 1.24)。

### 解决方案: conda hooks

创建 activate/deactivate 脚本实现自动隔离。

### 3.1 创建激活脚本

**路径**: `~/miniforge3/envs/yopo/etc/conda/activate.d/yopo_env_setup.sh`

```bash
#!/bin/bash
export _CONDA_YOPO_OLD_PYTHONPATH="$PYTHONPATH"
export _CONDA_YOPO_OLD_ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH"

CONDA_SITE_PACKAGES="$CONDA_PREFIX/lib/python3.8/site-packages"
export PYTHONPATH="$CONDA_SITE_PACKAGES"

[ -d "/opt/ros/noetic/lib/python3/dist-packages" ] && \
    export PYTHONPATH="$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages"
[ -d "/home/amov/catkin_ws/devel/lib/python3/dist-packages" ] && \
    export PYTHONPATH="$PYTHONPATH:/home/amov/catkin_ws/devel/lib/python3/dist-packages"
[ -d "/home/amov/Projects/YOPO/Controller/devel/lib/python3/dist-packages" ] && \
    export PYTHONPATH="$PYTHONPATH:/home/amov/Projects/YOPO/Controller/devel/lib/python3/dist-packages"

export ROS_PACKAGE_PATH="/home/amov/Projects/YOPO/Controller/src:/home/amov/Projects/YOPO/Simulator/src:${_CONDA_YOPO_OLD_ROS_PACKAGE_PATH}"
export YOPO_ROOT="/home/amov/Projects/YOPO"
```

### 3.2 创建退出脚本

**路径**: `~/miniforge3/envs/yopo/etc/conda/deactivate.d/yopo_env_cleanup.sh`

```bash
#!/bin/bash
[ -n "$_CONDA_YOPO_OLD_PYTHONPATH" ] && export PYTHONPATH="$_CONDA_YOPO_OLD_PYTHONPATH" || unset PYTHONPATH
[ -n "$_CONDA_YOPO_OLD_ROS_PACKAGE_PATH" ] && export ROS_PACKAGE_PATH="$_CONDA_YOPO_OLD_ROS_PACKAGE_PATH"
unset _CONDA_YOPO_OLD_PYTHONPATH _CONDA_YOPO_OLD_ROS_PACKAGE_PATH YOPO_ROOT
```

### 3.3 设置执行权限

```bash
chmod +x ~/miniforge3/envs/yopo/etc/conda/activate.d/yopo_env_setup.sh
chmod +x ~/miniforge3/envs/yopo/etc/conda/deactivate.d/yopo_env_cleanup.sh
```

---

## 阶段 4: 硬件集成

### 核心话题配置

| 发布节点 | 话题 | 订阅节点 |
|----------|------|----------|
| D435i | `/camera/depth/image_rect_raw` | YOPO 规划器 |
| D435i | `/camera/infra1/image_rect_raw` | VINS-Fusion |
| D435i | `/camera/infra2/image_rect_raw` | VINS-Fusion |
| D435i | `/camera/imu` | VINS-Fusion |
| VINS-Fusion | `/vins_estimator/imu_propagate` | YOPO, SO3 控制器 |
| YOPO | `/so3_control/pos_cmd` | SO3 控制器 |
| SO3 | `/mavros/setpoint_raw/attitude` | MAVROS |

### RealSense 配置参数

```yaml
depth_width: 480
depth_height: 270
depth_fps: 30
enable_infra1: true
enable_infra2: true
infra_width: 640
infra_height: 480
enable_gyro: true
enable_accel: true
unite_imu_method: "linear_interpolation"
```

### YOPO 规划器配置

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

---

## 阶段 5: 验证与运行

### 5.1 验证 conda 环境

```bash
conda activate yopo
python -c "import numpy; print(numpy.__version__, numpy.__file__)"
# 期望: 1.24.4 /home/amov/miniforge3/envs/yopo/...
```

### 5.2 验证 ROS 可用性

```bash
python -c "import rospy; print('rospy OK')"
rospack find realsense2_camera
rospack find vins
```

### 5.3 运行 YOPO

```bash
conda activate yopo
cd /home/amov/Projects/YOPO/YOPO
python test_yopo_ros.py --trial=1 --epoch=50
```

---

## 故障排查

| 问题 | 症状 | 解决方案 |
|------|------|----------|
| numpy 版本错误 | 显示 1.17.x | `conda deactivate && conda activate yopo` |
| rospy 无法导入 | ModuleNotFoundError | 检查 activate.d hook 脚本是否存在 |
| roslaunch 找不到 | command not found | 确保 `.bashrc` 有 `source /opt/ros/noetic/setup.bash` |
| OpenCV 符号错误 | undefined symbol | 添加 rosfix 函数和别名到 `.bashrc` |
| VIO 发散 | 位置跳变 > 1m | 确认使用 D435i IMU 而非 PX4 IMU |

---

## 详细参考文档

- 驱动安装详情: [driver-installation.md](driver-installation.md)
- 环境配置详情: [environment-setup.md](environment-setup.md)
- 硬件集成详情: [hardware-integration.md](hardware-integration.md)
