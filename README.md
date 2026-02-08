# YOPO 无人机自主导航系统 - 部署仓库

本仓库包含 YOPO 系统在 Jetson 平台的完整部署配置、文档和补丁。

已通过系统集成测试，剩余工作
* 搭建飞行平台
* PX4 IMU 外参标定
* SO3 控制器 hover_thrust 标定

## 仓库结构

```
├── docs/                    # 部署文档
├── configs/                 # 配置文件
├── patches/                 # 补丁文件
├── .cursor/skills/          # AI 辅助部署 Skill
├── third_party.md           # 第三方依赖克隆说明
└── README.md
```

## 快速开始

### 1. 克隆本仓库

```bash
git clone git@github.com:monitor666/YOPO_deploy.git ~/Projects
cd ~/Projects
```

### 2. 克隆第三方依赖

详见 [third_party.md](third_party.md)，主要包括：

```bash
# YOPO 主项目
git clone https://github.com/TJU-Aerial-Robotics/YOPO.git

# librealsense 
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense && git checkout v2.50.0

# ROS 工作空间
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone -b ros1-legacy https://github.com/IntelRealSense/realsense-ros.git
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
```

### 3. 安装 MAVROS (PX4 飞控通信)

```bash
# 安装 MAVROS 和相关工具
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras

# 安装地理数据库 (必须)
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 4. 应用补丁和配置

```bash
# VINS-Fusion OpenCV 4.x 兼容补丁 (已应用)
# 详见 patches/opencv4_compat.h

# ROS OpenCV 版本冲突修复
echo "source ~/Projects/configs/bash/rosfix.sh" >> ~/.bashrc
source ~/.bashrc

# conda 环境隔离 (yopo 环境创建后)
cp configs/conda/hooks/yopo_env_setup.sh ~/miniforge3/envs/yopo/etc/conda/activate.d/
cp configs/conda/hooks/yopo_env_cleanup.sh ~/miniforge3/envs/yopo/etc/conda/deactivate.d/
```

## 文档索引

| 文档 | 说明 |
|------|------|
| [IMU频率自动配置说明](docs/MAVROS_IMU频率自动配置说明.md) | YOPO/Controller/ 补丁和 IMU 频率自动配置 |
| [部署工作清单](docs/YOPO_部署工作清单.md) | 完整部署步骤和检查清单 |
| [硬件通信节点说明](docs/YOPO_硬件通信节点说明.md) | ROS 话题和节点通信规范 |
| [虚拟环境配置说明](docs/YOPO_虚拟环境与隔离配置说明.md) | conda 环境和 ROS 隔离配置 |
| [第三方依赖](third_party.md) | 依赖项目克隆和编译说明 |

## 第三方依赖

| 项目 | 版本 | 仓库/安装方式 |
|------|------|---------------|
| YOPO | master | https://github.com/TJU-Aerial-Robotics/YOPO |
| VINS-Fusion | master | https://github.com/HKUST-Aerial-Robotics/VINS-Fusion |
| realsense-ros | ros1-legacy | https://github.com/IntelRealSense/realsense-ros |
| librealsense | v2.50.0 | https://github.com/IntelRealSense/librealsense |
| MAVROS | noetic | `apt install ros-noetic-mavros ros-noetic-mavros-extras` |

## 关键路径

| 资源 | 路径 |
|------|------|
| catkin_ws | `/home/amov/Projects/catkin_ws` |
| YOPO 项目 | `/home/amov/Projects/YOPO` |
| Controller | `/home/amov/Projects/YOPO/Controller` |
| TensorRT 模型 | `/home/amov/Projects/YOPO/YOPO/saved/YOPO_1/yopo_trt.pth` |
| yopo 环境 | `/home/amov/miniforge3/envs/yopo` |

## 注意事项

- 第三方项目 (`YOPO/`, `catkin_ws/`, `librealsense/`) 不纳入版本控制
- 模型文件 (`*.pth`, `*.trt`) 不纳入版本控制
- 只提交 `docs/`, `configs/`, `patches/` 目录的内容
