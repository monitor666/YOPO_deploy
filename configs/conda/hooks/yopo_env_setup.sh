#!/bin/bash
# YOPO Conda 环境激活 Hook
# 在 conda activate yopo 时自动执行

# 保存原始环境变量 (用于 deactivate 时恢复)
export _CONDA_YOPO_OLD_PYTHONPATH="$PYTHONPATH"
export _CONDA_YOPO_OLD_ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH"

# 获取 conda 环境的 site-packages 路径
CONDA_SITE_PACKAGES="$CONDA_PREFIX/lib/python3.8/site-packages"

# 重新构建 PYTHONPATH: conda 优先，ROS 其次
export PYTHONPATH="$CONDA_SITE_PACKAGES"

# 添加 ROS 路径 (低优先级，放在 conda 之后)
if [ -d "/opt/ros/noetic/lib/python3/dist-packages" ]; then
    export PYTHONPATH="$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages"
fi

if [ -d "/home/amov/catkin_ws/devel/lib/python3/dist-packages" ]; then
    export PYTHONPATH="$PYTHONPATH:/home/amov/catkin_ws/devel/lib/python3/dist-packages"
fi

if [ -d "/home/amov/Projects/YOPO/Controller/devel/lib/python3/dist-packages" ]; then
    export PYTHONPATH="$PYTHONPATH:/home/amov/Projects/YOPO/Controller/devel/lib/python3/dist-packages"
fi

# 添加系统 TensorRT 路径 (Jetson 专用)
if [ -d "/usr/lib/python3.8/dist-packages/tensorrt" ]; then
    export PYTHONPATH="$PYTHONPATH:/usr/lib/python3.8/dist-packages"
fi

# 设置 YOPO 专用的 ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH="/home/amov/Projects/YOPO/Controller/src:/home/amov/Projects/YOPO/Simulator/src:${_CONDA_YOPO_OLD_ROS_PACKAGE_PATH}"

# 设置 YOPO 项目根目录
export YOPO_ROOT="/home/amov/Projects/YOPO"
