#!/bin/bash
# ============================================================
# YOPO 无人机自主导航系统 - 完整环境配置
# ============================================================
#
# 文件: ~/Projects/configs/bash/yopo_env.sh
# 作用: 一站式配置 YOPO 项目所需的全部环境
#
# ============================================================
# 安装方法 (推荐)
# ============================================================
#
# 在 ~/.bashrc 末尾添加以下两行:
#
#   # YOPO 项目环境配置
#   source ~/Projects/configs/bash/yopo_env.sh
#
# 添加后执行 source ~/.bashrc 或重新打开终端生效。
#
# ============================================================
# 包含的配置
# ============================================================
#
# 1. ROS 工作空间:
#    - catkin_ws (VINS-Fusion, yopo_tools)
#    - YOPO/Controller (so3_control, quadrotor_msgs)
#
# 2. OpenCV 版本冲突修复:
#    - rosfix 函数和相关 alias
#    - 解决 Jetson 上 OpenCV 4.2/4.5 符号冲突
#
# ============================================================

# ------------------------------------------------------------
# ROS 工作空间配置
# ------------------------------------------------------------

# 主 catkin 工作空间 (VINS-Fusion, yopo_tools 等)
if [ -f ~/Projects/catkin_ws/devel/setup.bash ]; then
    source ~/Projects/catkin_ws/devel/setup.bash
fi

# YOPO 控制器工作空间 (--extend 保留 catkin_ws 路径)
if [ -f ~/Projects/YOPO/Controller/devel/setup.bash ]; then
    source ~/Projects/YOPO/Controller/devel/setup.bash --extend
fi

# ------------------------------------------------------------
# OpenCV 版本冲突修复
# ------------------------------------------------------------
# 问题: Jetson 系统 OpenCV 4.2 和 4.5.4 共存导致符号冲突
# 表现: undefined symbol: _ZN2cv3MatC1Ev
# 解决: 使用 LD_PRELOAD 强制加载 4.5 版本

rosfix() {
    LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.5:/usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.5" "$@"
}

# ROS 命令 alias (自动使用 rosfix)
alias roslaunch='rosfix roslaunch'
alias rosrun='rosfix rosrun'
alias roscore='rosfix roscore'
alias rviz='rosfix rviz'
alias rqt='rosfix rqt'
alias rosbag='rosfix rosbag'

# ------------------------------------------------------------
# 便捷命令
# ------------------------------------------------------------

# 快速启动 YOPO 相关服务的 alias
alias yopo-mavros='roslaunch ~/Projects/configs/mavros/px4_yopo.launch fcu_url:=/dev/ttyACM0:921600'
alias yopo-camera='roslaunch ~/Projects/configs/realsense/yopo_d455f_camera.launch'
alias yopo-vins='rosrun vins vins_node ~/Projects/configs/vins/realsense_d435i/realsense_stereo_imu_config.yaml'
alias yopo-so3='roslaunch so3_control controller_network.launch'
alias yopo-planner='cd ~/Projects/YOPO/YOPO && conda activate yopo && python test_yopo_ros.py --use_tensorrt=1'

# 起飞/降落服务
alias yopo-takeoff='rosservice call /network_controller_node/takeoff_land "{takeoff: true, takeoff_altitude: 1.5}"'
alias yopo-land='rosservice call /network_controller_node/takeoff_land "{takeoff: false, takeoff_altitude: 0.0}"'

# 话题检查
alias yopo-check='rostopic list | grep -E "(camera|vins|mavros|so3_control)"'
alias yopo-hz-imu='rostopic hz /mavros/imu/data_raw'
alias yopo-hz-depth='rostopic hz /camera/depth/image_rect_raw'
alias yopo-hz-odom='rostopic hz /vins_estimator/imu_propagate'
alias yopo-hz-cmd='rostopic hz /so3_control/pos_cmd'
alias yopo-hz-att='rostopic hz /mavros/setpoint_raw/attitude'

# ============================================================
# 验证配置
# ============================================================
#
# 运行以下命令验证配置正确:
#
#   # 检查 ROS 路径
#   echo $ROS_PACKAGE_PATH
#
#   # 检查关键包
#   rospack find yopo_tools     # catkin_ws
#   rospack find so3_control    # YOPO/Controller
#
#   # 查看可用的 yopo 命令
#   alias | grep yopo
#
# ============================================================
# 快捷命令说明
# ============================================================
#
# 启动命令 (按顺序执行):
#   yopo-mavros    - 启动 MAVROS (PX4 通信)
#   yopo-camera    - 启动 RealSense D455f
#   yopo-vins      - 启动 VINS-Fusion
#   yopo-so3       - 启动 SO3 控制器
#   yopo-planner   - 启动 YOPO 规划器
#
# 控制命令:
#   yopo-takeoff   - 调用起飞服务
#   yopo-land      - 调用降落服务
#
# 检查命令:
#   yopo-check     - 列出所有 YOPO 相关话题
#   yopo-hz-imu    - 检查 IMU 频率 (应 >= 200Hz)
#   yopo-hz-depth  - 检查深度图频率 (应 ~30Hz)
#   yopo-hz-odom   - 检查里程计频率 (应 >= 100Hz)
#   yopo-hz-cmd    - 检查位置命令频率 (应 ~50Hz)
#   yopo-hz-att    - 检查姿态指令频率 (应 ~50Hz)
#
# ============================================================
