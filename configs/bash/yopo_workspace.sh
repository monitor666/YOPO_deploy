# ============================================================
# YOPO 无人机自主导航系统 - ROS 工作空间配置
# ============================================================
#
# 文件: ~/Projects/configs/bash/yopo_workspace.sh
# 作用: 配置 YOPO 项目所需的 ROS 工作空间和环境变量
#
# ============================================================
# 安装方法
# ============================================================
#
# 方法1: 在 ~/.bashrc 末尾添加一行 (推荐):
#
#   source ~/Projects/configs/bash/yopo_workspace.sh
#
# 方法2: 手动复制下方内容到 ~/.bashrc 末尾
#
# 添加后执行 source ~/.bashrc 或重新打开终端生效。
#
# ============================================================
# 包含的工作空间
# ============================================================
#
# 1. catkin_ws      - 主工作空间，包含:
#                     - VINS-Fusion (VIO)
#                     - yopo_tools (IMU 频率配置等工具)
#
# 2. YOPO/Controller - YOPO 控制器工作空间，包含:
#                     - so3_control (SO3 位置控制器)
#                     - quadrotor_msgs (消息定义)
#                     - so3_quadrotor_simulator (仿真器)
#
# ============================================================
# 注意事项
# ============================================================
#
# - 必须先 source catkin_ws，再 source YOPO/Controller
# - 使用 --extend 选项让两个工作空间共存
# - 如果遇到包找不到的问题，检查 source 顺序
#
# ============================================================

# 主 catkin 工作空间 (VINS-Fusion, yopo_tools 等)
if [ -f ~/Projects/catkin_ws/devel/setup.bash ]; then
    source ~/Projects/catkin_ws/devel/setup.bash
fi

# YOPO 控制器工作空间 (--extend 保留 catkin_ws 路径)
if [ -f ~/Projects/YOPO/Controller/devel/setup.bash ]; then
    source ~/Projects/YOPO/Controller/devel/setup.bash --extend
fi

# ============================================================
# 验证工作空间配置
# ============================================================
#
# 运行以下命令验证两个工作空间都被正确加载:
#
#   echo $ROS_PACKAGE_PATH
#
# 应该看到类似输出 (包含两个工作空间):
#   /home/amov/Projects/YOPO/Controller/src:
#   /home/amov/Projects/catkin_ws/src:
#   /opt/ros/noetic/share
#
# 验证关键包能被找到:
#
#   rospack find yopo_tools     # 应返回 catkin_ws 中的路径
#   rospack find so3_control    # 应返回 YOPO/Controller 中的路径
#
# ============================================================
