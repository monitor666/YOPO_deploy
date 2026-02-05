# ============================================================
# ROS OpenCV 4.2/4.5 版本冲突修复
# ============================================================
#
# 问题说明:
# Jetson 系统存在 OpenCV 4.2 和 4.5.4 共存的情况：
# - ROS cv_bridge 依赖 OpenCV 4.2
# - 系统头文件是 4.5.4 版本
# - 编译时用 4.5 头文件，运行时链接 4.2 库，导致符号未定义错误
#
# 错误表现:
# undefined symbol: _ZN2cv3MatC1Ev
#
# ============================================================
# 安装方法
# ============================================================
#
# 将以下内容添加到 ~/.bashrc 末尾：
#
#   source /home/amov/Projects/configs/bash/rosfix.sh
#
# 或者直接复制下方内容到 ~/.bashrc 末尾。
# 添加后执行 source ~/.bashrc 或重新打开终端。
#
# ============================================================

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
