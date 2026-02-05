# 第三方依赖说明

本文档记录 YOPO 部署所需的第三方项目及其版本。

## 克隆命令

### YOPO 主项目
```bash
git clone https://github.com/TJU-Aerial-Robotics/YOPO.git
```

### ROS 工作空间依赖
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src

# realsense-ros (必须使用 ros1-legacy 分支)
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros && git checkout ros1-legacy && cd ..

# VINS-Fusion
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
```

### librealsense
```bash
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense && git checkout v2.50.0
```

## 版本锁定

| 项目 | 版本/分支 | Commit (可选) |
|------|-----------|---------------|
| YOPO | master | - |
| VINS-Fusion | master | - |
| realsense-ros | ros1-legacy | - |
| librealsense | v2.50.0 | - |

## 补丁说明

VINS-Fusion 需要应用 OpenCV 4.x 兼容补丁，详见 `patches/` 目录。
