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

---

## PX4 驱动安装 (MAVROS)

> **说明**: YOPO 使用 PX4 飞控 IMU 作为 VINS-Fusion 输入，需要安装 MAVROS 与 PX4 通信。

### 安装 MAVROS

```bash
# 安装 MAVROS 及其附加功能包
sudo apt-get update
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras

# 安装地理数据库 (必须，否则 MAVROS 无法启动)
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 验证安装

```bash
# 检查 MAVROS 包是否安装成功
rospack find mavros
rospack find mavros_extras

# 应输出类似:
# /opt/ros/noetic/share/mavros
# /opt/ros/noetic/share/mavros_extras
```

### PX4 飞控连接配置

#### USB 连接

```bash
# 检查设备
ls /dev/ttyACM*

# 设置 USB 设备权限 (首次使用需要)
sudo usermod -a -G dialout $USER
# 重新登录或重启后生效

# 启动 MAVROS (USB)
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
```

#### 串口连接 (Jetson UART)

```bash
# 检查设备
ls /dev/ttyTHS*

# Jetson 串口权限设置
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyTHS0  # 临时设置

# 启动 MAVROS (串口)
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS0:921600
```

#### 自定义 Launch 文件 (可选)

创建 `~/Projects/configs/mavros/px4_yopo.launch`:

```xml
<launch>
    <!-- MAVROS 节点 -->
    <arg name="fcu_url" default="/dev/ttyACM0:921600"/>
    <arg name="gcs_url" default=""/>
    <arg name="tgt_system" default="1"/>
    <arg name="tgt_component" default="1"/>

    <include file="$(find mavros)/launch/px4.launch">
        <arg name="fcu_url" value="$(arg fcu_url)"/>
        <arg name="gcs_url" value="$(arg gcs_url)"/>
        <arg name="tgt_system" value="$(arg tgt_system)"/>
        <arg name="tgt_component" value="$(arg tgt_component)"/>
    </include>
</launch>
```

### 验证 PX4 连接

```bash
# 启动 MAVROS
roslaunch ~/Projects/configs/mavros/px4_yopo.launch fcu_url:=/dev/ttyACM0:921600

# 在另一个终端检查连接状态
rostopic echo /mavros/state

# 期望输出:
# ---
# header:
#   stamp: ...
#   frame_id: ''
# connected: True      <-- 关键: 必须为 True
# armed: False
# guided: False
# manual_input: True
# mode: "MANUAL"       <-- 或其他模式
```

### 验证 IMU 数据

```bash
# 检查 IMU 话题频率 (应 >= 200Hz)
rostopic hz /mavros/imu/data_raw

# 查看 IMU 数据
rostopic echo /mavros/imu/data_raw --noarr

# 期望输出:
# ---
# header:
#   stamp: ...
#   frame_id: "base_link"
# orientation: ...
# angular_velocity: ...
# linear_acceleration: ...
```

### PX4 参数配置 (QGroundControl)

为确保 IMU 数据正常发布，需要在 QGroundControl 中检查以下 PX4 参数:

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `MAV_0_RATE` | 0 (不限制) | MAVLink 消息发送速率 |
| `IMU_GYRO_RATEMAX` | 400 | 陀螺仪最大输出频率 |
| `IMU_INTEG_RATE` | 400 | IMU 积分频率 |
| `SER_TEL1_BAUD` | 921600 | 串口波特率 (如使用 TELEM1) |

### OFFBOARD 模式控制

```bash
# 设置 OFFBOARD 模式
rosservice call /mavros/set_mode "custom_mode: 'OFFBOARD'"

# 解锁电机
rosservice call /mavros/cmd/arming "value: true"

# 上锁电机
rosservice call /mavros/cmd/arming "value: false"
```

### 故障排查

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| `connected: False` | 串口配置错误 | 检查 fcu_url 参数和波特率 |
| 无 IMU 数据 | MAVLink 消息未开启 | 检查 PX4 的 MAV_0_RATE 参数 |
| IMU 频率过低 | 波特率不足 | 提高串口波特率到 921600 |
| 权限拒绝 | 用户不在 dialout 组 | 执行 `sudo usermod -a -G dialout $USER` |
| Jetson 串口无响应 | 串口未启用 | 检查 Jetson 设备树配置 |

---

## 版本锁定

| 项目 | 版本/分支 | 原因 |
|------|-----------|---------------|
| YOPO | master | - |
| librealsense | v2.50.0 | 与 realsense-ros ros1-legacy 分支兼容 |
| VINS-Fusion | master | 支持 D455f 双目红外 + PX4 IMU 输入 |
| realsense-ros | ros1-legacy | ROS Noetic + ROS1 兼容 |
| MAVROS | apt 默认版本 | ROS Noetic 官方仓库 |

## 验证命令

```bash
# 验证 librealsense 安装
realsense-viewer  # 或 rs-enumerate-devices

# 验证 ROS 包安装
rospack find realsense2_camera
rospack find vins
rospack find cv_bridge
rospack find mavros

# 验证 MAVROS (需要连接飞控)
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
rostopic echo /mavros/state

# 验证 RealSense 话题（需要连接相机）
# 本机 OpenCV 4.2 和 4.5.4 共存，编译使用 4.5 头文件但运行链接 4.2 库导致的符号未定义报错， `roslaunch`、`rosrun` 不能正常使用。修复见下方"OpenCV 4.2/4.5 版本冲突"
roslaunch realsense2_camera rs_camera.launch
rostopic list | grep camera
```

## 补丁说明

VINS-Fusion 需要应用 OpenCV 4.x 兼容补丁，详见 `patches/` 目录。

## OpenCV 4.2/4.5 版本冲突

Jetson 系统 OpenCV 版本冲突导致 ROS 命令报符号未定义错误，需添加 rosfix 修复，详见 `configs/bash/rosfix.sh`。
