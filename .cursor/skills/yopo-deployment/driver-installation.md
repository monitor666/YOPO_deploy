# YOPO 驱动安装参考

> **状态**: ✅ 已完成 (2026-02-04)

本文档为驱动安装的参考备份。如需重新安装，请参考 `/home/amov/Projects/third_party.md`。

## 已安装版本

| 驱动 | 版本 | 状态 |
|------|------|------|
| librealsense | 2.50.0 | ✅ 源码编译安装 |
| realsense-ros | ros1-legacy 分支 | ✅ catkin_ws 中 |
| VINS-Fusion | master 分支 | ✅ OpenCV 4.x 兼容已修复 |
| cv_bridge | 1.16.2 (系统) | ✅ ROS Noetic 自带 |

## 关键路径

| 资源 | 路径 |
|------|------|
| catkin_ws | `/home/amov/Projects/catkin_ws` |
| realsense-ros | `/home/amov/Projects/catkin_ws/src/realsense-ros` |
| VINS-Fusion | `/home/amov/Projects/catkin_ws/src/VINS-Fusion` |
| OpenCV 4.x 补丁 | `/home/amov/Projects/patches/opencv4_compat.h` |
| 第三方依赖说明 | `/home/amov/Projects/third_party.md` |

## OpenCV 4.x 兼容性修复

VINS-Fusion 源文件已添加兼容补丁引用：

```cpp
#include "/home/amov/Projects/patches/opencv4_compat.h"
```

**已修改的文件**:
- `vins_estimator/src/featureTracker/feature_tracker.cpp`
- `camera_models/src/chessboard/Chessboard.cc`
- `camera_models/src/calib/CameraCalibration.cc`
- `camera_models/src/intrinsic_calib.cc`
- `loop_fusion/src/pose_graph.cpp`
- `loop_fusion/src/keyframe.cpp`
- `loop_fusion/src/ThirdParty/DVision/BRIEF.cpp`
- `vins_estimator/src/KITTIGPSTest.cpp`
- `vins_estimator/src/KITTIOdomTest.cpp`

**还原命令** (如需恢复原始代码):
```bash
cd /home/amov/Projects/catkin_ws/src/VINS-Fusion
git checkout .
```

## OpenCV 4.2/4.5 版本冲突修复

已在 `~/.bashrc` 中配置 rosfix 函数：

```bash
rosfix() {
    LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.5" "$@"
}
alias roslaunch='rosfix roslaunch'
alias rosrun='rosfix rosrun'
alias rviz='rosfix rviz'
```

## 验证命令

```bash
# 验证 librealsense
realsense-viewer  # 或 rs-enumerate-devices

# 验证 ROS 包
rospack find realsense2_camera
rospack find vins
```

## 重新安装指南

如需重新安装驱动，参考 `/home/amov/Projects/third_party.md` 中的克隆和编译命令。
