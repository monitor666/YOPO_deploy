# YOPO 驱动版本说明

> **文档用途**: 本文档记录 YOPO 项目所需驱动的版本和安装命令。AI 助手在协助安装驱动时必须遵循本文档中的版本要求。

---

## 元信息

| 属性 | 值 |
|------|-----|
| 目标平台 | NVIDIA Jetson (aarch64) |
| 系统内核 | 5.10.216-tegra |
| ROS 版本 | Noetic |
| CUDA 版本 | 11.4 |
| OpenCV 版本 | 4.2 / 4.5.4 共存（需特殊处理） |

---

## 驱动列表

### 1. librealsense (Intel RealSense SDK)

| 属性 | 值 |
|------|-----|
| **版本** | 2.50.0 |
| **仓库** | https://github.com/IntelRealSense/librealsense.git |
| **分支/标签** | v2.50.0 |
| **编译选项** | CUDA 支持 (`-DBUILD_WITH_CUDA=true`) |

#### 安装命令


```bash
# 1. 克隆源码
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.50.0

# 2. 安装依赖（已安装，可跳过）
# sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
# sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# 3. 编译 (Jetson 需要 CUDA 支持)
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=true -DBUILD_WITH_CUDA=true -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig

# 4. udev 规则 (已存在，可跳过) =====
# sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
# sudo udevadm control --reload-rules && sudo udevadm trigger
```
---

### 2. realsense-ros (RealSense ROS 驱动)

| 属性 | 值 |
|------|-----|
| **ROS 包名** | realsense2_camera |
| **仓库** | https://github.com/IntelRealSense/realsense-ros.git |
| **分支** | ros1-legacy |
| **依赖** | librealsense 2.50.0 |

#### 安装命令

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout ros1-legacy
```

---

### 3. VINS-Fusion (视觉惯性里程计)

| 属性 | 值 |
|------|-----|
| **仓库** | https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git |
| **分支** | master (默认) |
| **依赖** | OpenCV, Ceres Solver, Eigen3 |

#### 安装命令

```bash
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
```

#### 前置依赖

```bash
# 安装 VINS 依赖 (已安装，可跳过)
# sudo apt-get install libceres-dev libeigen3-dev
```

#### OpenCV 4.x 兼容性修复

VINS-Fusion 原始代码使用 OpenCV 3.x API，需要添加兼容性宏（源码已修改）。

涉及修改的文件：
- `camera_models/src/chessboard/Chessboard.cc`
- `camera_models/src/calib/CameraCalibration.cc`
- `camera_models/src/intrinsic_calib.cc`
- `loop_fusion/src/pose_graph.cpp`
- `loop_fusion/src/keyframe.cpp`
- `loop_fusion/src/ThirdParty/DVision/BRIEF.cpp`
- `vins_estimator/src/featureTracker/feature_tracker.cpp`
- `vins_estimator/src/KITTIGPSTest.cpp`
- `vins_estimator/src/KITTIOdomTest.cpp`

> **还原方法**: `cd ~/Projects/catkin_ws/src/VINS-Fusion && git checkout .`

---

### 4. vision_opencv (cv_bridge) - 系统已有

| 属性 | 值 |
|------|-----|
| **ROS 包名** | cv_bridge, image_geometry |
| **系统路径** | /opt/ros/noetic/share/cv_bridge |
| **已安装版本** | 1.16.2 |
| **状态** | ✅ **系统自带，无需安装** |

> **说明**: ROS Noetic 桌面完整版 (`ros-noetic-desktop-full`) 已包含 `cv_bridge`，无需额外编译安装。

---

## 安装清单摘要

| 驱动 | 需要安装？ | 安装方式 |
|------|-----------|----------|
| librealsense | ✅ 需要 | 源码编译 (v2.50.0) |
| realsense-ros | ✅ 需要 | 克隆到 catkin_ws (ros1-legacy 分支) |
| VINS-Fusion | ✅ 需要 | 克隆到 catkin_ws |
| vision_opencv | ❌ 不需要 | 系统已有 |

---

## 编译 catkin_ws

```bash
cd ~/Projects/catkin_ws
catkin_make
```

## 添加到 bashrc

```bash
# 加载 ROS 工作空间
echo "source ~/Projects/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> **注意**: 还需要添加 OpenCV 冲突修复，见下方"已知问题与解决方案"章节。

## 版本兼容性约束

| 组件 | 版本要求 | 原因 |
|------|----------|------|
| librealsense | 2.50.0 | 与 realsense-ros ros1-legacy 分支兼容 |
| realsense-ros | ros1-legacy 分支 | ROS Noetic + ROS1 兼容 |
| VINS-Fusion | master | 支持 D435i 双目红外 + IMU 输入 |
| vision_opencv | 系统自带 1.16.2 | ROS Noetic 已包含 |

---

## 已知问题与解决方案

### OpenCV 4.2/4.5 版本冲突

**问题描述**: Jetson 系统存在 OpenCV 4.2 和 4.5.4 共存。编译时使用 4.5 头文件，运行时链接 4.2 库，导致运行 ROS 节点时报错：

```
undefined symbol: _ZN2cv3MatC1Ev
```

**原因**: ROS Noetic 的 cv_bridge 依赖 OpenCV 4.2，但系统头文件是 4.5.4 版本，两者符号导出不一致。

**解决方案**: 在 `~/.bashrc` 中添加以下内容：

```bash
# ROS with OpenCV fix - use this function to run ROS commands
rosfix() {
    LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.5" "$@"
}

# ROS command aliases with OpenCV fix
alias roslaunch='rosfix roslaunch'
alias rosrun='rosfix rosrun'
alias roscore='rosfix roscore'
alias rviz='rosfix rviz'
alias rqt='rosfix rqt'
alias rosbag='rosfix rosbag'
```

添加后重新打开终端即可正常使用 `roslaunch`、`rosrun` 等命令。

---

## 验证命令

```bash
# 验证 librealsense 安装
realsense-viewer  # 或 rs-enumerate-devices

# 验证 ROS 包安装
rospack find realsense2_camera
rospack find vins
rospack find cv_bridge

# 验证 RealSense 话题（需要连接相机）
roslaunch realsense2_camera rs_camera.launch
rostopic list | grep camera
```
