#!/bin/bash
# YOPO Conda 环境退出 Hook
# 在 conda deactivate 时自动执行

# 恢复原始 PYTHONPATH
if [ -n "$_CONDA_YOPO_OLD_PYTHONPATH" ]; then
    export PYTHONPATH="$_CONDA_YOPO_OLD_PYTHONPATH"
    unset _CONDA_YOPO_OLD_PYTHONPATH
else
    unset PYTHONPATH
fi

# 恢复原始 ROS_PACKAGE_PATH
if [ -n "$_CONDA_YOPO_OLD_ROS_PACKAGE_PATH" ]; then
    export ROS_PACKAGE_PATH="$_CONDA_YOPO_OLD_ROS_PACKAGE_PATH"
    unset _CONDA_YOPO_OLD_ROS_PACKAGE_PATH
fi

# 清理 YOPO 相关变量
unset YOPO_ROOT
