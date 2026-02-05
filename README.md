# YOPO 无人机自主导航系统 - 部署仓库

本仓库包含 YOPO 系统在 Jetson 平台的完整部署配置、文档和脚本。

## 仓库结构

```
├── docs/           # 部署文档
├── configs/        # 配置文件
├── scripts/        # 部署脚本
├── patches/        # 补丁文件
└── .cursor/skills/ # AI 辅助部署
```

## 快速开始

### 1. 克隆仓库

```bash
git clone git@github.com:monitor666/YOPO_deploy.git ~/Projects
cd ~/Projects
```

### 2. 克隆第三方依赖

```bash
# YOPO 主项目
git clone https://github.com/TJU-Aerial-Robotics/YOPO.git

# ROS 工作空间
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone -b ros1-legacy https://github.com/IntelRealSense/realsense-ros.git
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
cd ~/Projects

# librealsense (如需编译)
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense && git checkout v2.50.0 && cd ..
```

### 3. 查看部署文档

- [硬件通信节点说明](docs/YOPO_硬件通信节点说明.md)
- [虚拟环境配置说明](docs/YOPO_虚拟环境与隔离配置说明.md)
- [驱动版本说明](docs/YOPO_驱动版本说明.md)
- [部署工作清单](docs/YOPO_部署工作清单.md)

## 第三方依赖

| 项目 | 版本 | 仓库 |
|------|------|------|
| YOPO | master | https://github.com/TJU-Aerial-Robotics/YOPO |
| VINS-Fusion | master | https://github.com/HKUST-Aerial-Robotics/VINS-Fusion |
| realsense-ros | ros1-legacy | https://github.com/IntelRealSense/realsense-ros |
| librealsense | v2.50.0 | https://github.com/IntelRealSense/librealsense |

## 注意事项

- 第三方项目 (YOPO/, catkin_ws/, librealsense/) 不纳入版本控制
- 模型文件 (*.pth, *.trt) 不纳入版本控制
- 只提交 docs/, configs/, scripts/, patches/ 目录的内容
