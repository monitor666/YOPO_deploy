# YOPO 驱动安装详情

## 驱动版本要求

| 驱动 | 版本 | 仓库/分支 |
|------|------|-----------|
| librealsense | 2.50.0 | https://github.com/IntelRealSense/librealsense.git (v2.50.0) |
| realsense-ros | ros1-legacy | https://github.com/IntelRealSense/realsense-ros.git |
| VINS-Fusion | master | https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git |
| cv_bridge | 1.16.2 | 系统自带，无需安装 |

## 1. librealsense 完整安装

### 前置依赖

```bash
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```

### 编译安装

```bash
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.50.0

mkdir build && cd build
cmake .. \
    -DBUILD_EXAMPLES=true \
    -DBUILD_WITH_CUDA=true \
    -DCMAKE_BUILD_TYPE=Release

make -j$(nproc)
sudo make install
sudo ldconfig
```

### udev 规则 (如未安装)

```bash
sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 验证安装

```bash
realsense-viewer
# 或
rs-enumerate-devices
```

---

## 2. realsense-ros 安装

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout ros1-legacy
```

**重要**: 必须使用 `ros1-legacy` 分支以兼容 ROS Noetic。

---

## 3. VINS-Fusion 安装

### 克隆仓库

```bash
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
```

### 前置依赖

```bash
sudo apt-get install libceres-dev libeigen3-dev
```

### OpenCV 4.x 兼容性修复

VINS-Fusion 原始代码使用 OpenCV 3.x API，需在以下文件开头添加兼容性宏：

**需要修改的文件列表**:

1. `camera_models/src/chessboard/Chessboard.cc`
2. `camera_models/src/calib/CameraCalibration.cc`
3. `camera_models/src/intrinsic_calib.cc`
4. `loop_fusion/src/pose_graph.cpp`
5. `loop_fusion/src/keyframe.cpp`
6. `loop_fusion/src/ThirdParty/DVision/BRIEF.cpp`
7. `vins_estimator/src/featureTracker/feature_tracker.cpp`
8. `vins_estimator/src/KITTIGPSTest.cpp`
9. `vins_estimator/src/KITTIOdomTest.cpp`

**在每个文件的 `#include` 之后添加**:

```cpp
// OpenCV 4.x 兼容性宏
#if CV_MAJOR_VERSION >= 4
#define CV_LOAD_IMAGE_GRAYSCALE cv::IMREAD_GRAYSCALE
#define CV_LOAD_IMAGE_COLOR cv::IMREAD_COLOR
#define CV_LOAD_IMAGE_UNCHANGED cv::IMREAD_UNCHANGED
#define CV_BGR2GRAY cv::COLOR_BGR2GRAY
#define CV_GRAY2BGR cv::COLOR_GRAY2BGR
#define CV_RGB2GRAY cv::COLOR_RGB2GRAY
#define CV_CALIB_CB_ADAPTIVE_THRESH cv::CALIB_CB_ADAPTIVE_THRESH
#define CV_CALIB_CB_NORMALIZE_IMAGE cv::CALIB_CB_NORMALIZE_IMAGE
#define CV_CALIB_CB_FILTER_QUADS cv::CALIB_CB_FILTER_QUADS
#define CV_CALIB_CB_FAST_CHECK cv::CALIB_CB_FAST_CHECK
#define CV_TERMCRIT_EPS cv::TermCriteria::EPS
#define CV_TERMCRIT_ITER cv::TermCriteria::MAX_ITER
#define CV_FM_RANSAC cv::FM_RANSAC
#define CV_FONT_HERSHEY_SIMPLEX cv::FONT_HERSHEY_SIMPLEX
#define CV_AA cv::LINE_AA
#endif
```

**还原方法** (如需恢复原始代码):

```bash
cd ~/catkin_ws/src/VINS-Fusion
git checkout .
```

---

## 4. OpenCV 4.2/4.5 版本冲突修复

### 问题描述

Jetson 系统存在 OpenCV 4.2 和 4.5.4 共存：
- ROS cv_bridge 依赖 OpenCV 4.2
- 系统头文件是 4.5.4 版本
- 编译时用 4.5 头文件，运行时链接 4.2 库，导致符号错误

### 错误表现

```
undefined symbol: _ZN2cv3MatC1Ev
```

### 解决方案

在 `~/.bashrc` 末尾添加：

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

添加后执行 `source ~/.bashrc` 或重新打开终端。

---

## 5. 编译 catkin_ws

```bash
cd ~/catkin_ws
catkin_make
```

### 添加到 bashrc

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 6. 验证安装

```bash
# 验证 librealsense
realsense-viewer

# 验证 ROS 包
rospack find realsense2_camera
rospack find vins
rospack find cv_bridge

# 验证 RealSense 话题 (需要连接相机)
roslaunch realsense2_camera rs_camera.launch
rostopic list | grep camera
```

---

## 版本兼容性说明

| 组件 | 版本要求 | 原因 |
|------|----------|------|
| librealsense | 2.50.0 | 与 realsense-ros ros1-legacy 分支兼容 |
| realsense-ros | ros1-legacy | ROS Noetic + ROS1 兼容 |
| VINS-Fusion | master | 支持 D435i 双目红外 + IMU 输入 |
| cv_bridge | 系统自带 1.16.2 | ROS Noetic 已包含 |
