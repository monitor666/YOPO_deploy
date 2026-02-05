# 第三方依赖说明

本文档记录 YOPO 部署所需的第三方项目及其版本。

## 克隆命令

### YOPO 主项目
```bash
git clone https://github.com/TJU-Aerial-Robotics/YOPO.git
```

### librealsense
```bash
# 克隆源码 (使用 v2.50.0 分支)
cd ~/Projects
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense && git checkout v2.50.0

# 安装依赖
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# 编译
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=true -DBUILD_WITH_CUDA=true -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig

# udev 规则
sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### ROS 工作空间依赖 (编译完 librealsense 2.50.0 之后)
```bash
cd ~/Projects
mkdir -p catkin_ws/src
cd catkin_ws/src

# realsense-ros (必须使用 ros1-legacy 分支)
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros && git checkout ros1-legacy && cd ..

# VINS-Fusion (使用 master 默认分支)
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
# 安装 VINS 依赖 
sudo apt-get install libceres-dev libeigen3-dev

# 编译 catkin_ws
# 注意: 编译前还需要添加 OpenCV 冲突修复，见下方"补丁说明"
cd ~/Projects/catkin_ws
catkin_make

# 添加到 bashrc
echo "source ~/Projects/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 版本锁定

| 项目 | 版本/分支 | 原因 |
|------|-----------|---------------|
| YOPO | master | - |
| librealsense | v2.50.0 | 与 realsense-ros ros1-legacy 分支兼容 |
| VINS-Fusion | master | 支持 D455f 双目红外 + IMU 输入 |
| realsense-ros | ros1-legacy | ROS Noetic + ROS1 兼容 |

## 验证命令

```bash
# 验证 librealsense 安装
realsense-viewer  # 或 rs-enumerate-devices

# 验证 ROS 包安装
rospack find realsense2_camera
rospack find vins
rospack find cv_bridge

# 验证 RealSense 话题（需要连接相机）
# 本机 OpenCV 4.2 和 4.5.4 共存，编译使用 4.5 头文件但运行链接 4.2 库导致的符号未定义报错， `roslaunch`、`rosrun` 不能正常使用。修复见下方"OpenCV 4.2/4.5 版本冲突"
roslaunch realsense2_camera rs_camera.launch
rostopic list | grep camera
```

## 补丁说明

VINS-Fusion 需要应用 OpenCV 4.x 兼容补丁，详见 `patches/` 目录。

## OpenCV 4.2/4.5 版本冲突

Jetson 系统 OpenCV 版本冲突导致 ROS 命令报符号未定义错误，需添加 rosfix 修复，详见 `configs/bash/rosfix.sh`。