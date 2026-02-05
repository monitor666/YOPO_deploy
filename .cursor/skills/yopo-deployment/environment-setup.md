# YOPO 环境配置参考

> **状态**: ✅ 已完成 (2026-02-04)

本文档为 conda 环境和 ROS 隔离配置的参考备份。

## 环境信息

| 配置项 | 值 |
|--------|-----|
| conda 发行版 | miniforge3 |
| 环境名 | `yopo` |
| Python 版本 | 3.8.20 |
| 环境路径 | `/home/amov/miniforge3/envs/yopo` |

## 核心依赖

| 包名 | 版本 | 说明 |
|------|------|------|
| torch | 2.0.0a0+nv23.3 | Jetson 专用，**禁止升级** |
| torchvision | 0.15.1 | Jetson 专用 |
| torch2trt | 0.5.0 | TensorRT 转换 |
| numpy | 1.24.4 | 优先级高于 ROS numpy |
| opencv-python-headless | 4.12.0.88 | 无 GUI 依赖 |
| open3d | 0.18.0 | 3D 点云处理 |

## ROS 隔离配置

### 配置文件位置

| 文件 | 路径 |
|------|------|
| 激活脚本 | `~/miniforge3/envs/yopo/etc/conda/activate.d/yopo_env_setup.sh` |
| 退出脚本 | `~/miniforge3/envs/yopo/etc/conda/deactivate.d/yopo_env_cleanup.sh` |
| 配置源文件 | `/home/amov/Projects/configs/conda/hooks/` |

### PYTHONPATH 优先级

```
优先级 1 (最高): conda site-packages (numpy 1.24.4, torch 等)
优先级 2:        /opt/ros/noetic/lib/python3/dist-packages
优先级 3:        /home/amov/catkin_ws/devel/lib/python3/dist-packages
优先级 4:        /home/amov/Projects/YOPO/Controller/devel/lib/python3/dist-packages
```

## 使用方法

```bash
# 激活环境 (自动执行隔离)
conda activate yopo

# 验证 numpy 来源
python -c "import numpy; print(numpy.__version__, numpy.__file__)"
# 期望: 1.24.4 /home/amov/miniforge3/envs/yopo/...

# 退出环境 (自动恢复)
conda deactivate
```

## 恢复 conda hooks

如果 hooks 脚本丢失（如重装 miniforge3 后）：

```bash
# 创建目录
mkdir -p ~/miniforge3/envs/yopo/etc/conda/activate.d
mkdir -p ~/miniforge3/envs/yopo/etc/conda/deactivate.d

# 从源文件复制
cp /home/amov/Projects/configs/conda/hooks/yopo_env_setup.sh \
   ~/miniforge3/envs/yopo/etc/conda/activate.d/
cp /home/amov/Projects/configs/conda/hooks/yopo_env_cleanup.sh \
   ~/miniforge3/envs/yopo/etc/conda/deactivate.d/

# 设置执行权限
chmod +x ~/miniforge3/envs/yopo/etc/conda/activate.d/yopo_env_setup.sh
chmod +x ~/miniforge3/envs/yopo/etc/conda/deactivate.d/yopo_env_cleanup.sh
```

## 重建环境

如需完全重建 yopo 环境：

```bash
# 删除旧环境
conda remove -n yopo --all

# 创建新环境
conda create -n yopo python=3.8 -y
conda activate yopo

# 安装 Jetson 专用 PyTorch (从 NVIDIA 预装或 wheel 安装)
# 禁止从 PyPI 安装 CUDA 版本的 torch/torchvision

# 安装其他依赖
pip install numpy==1.24.4 scipy==1.10.1 scikit-learn==1.3.2
pip install opencv-python-headless==4.12.0.88 open3d==0.18.0 pillow==10.4.0
pip install ruamel.yaml==0.17.21 tensorboard==2.14.0 rich scikit-build==0.18.1 empy==4.2
pip install catkin_pkg rospkg netifaces torch2trt==0.5.0

# 恢复 conda hooks (见上方)
```

## 故障排查

| 问题 | 解决方案 |
|------|----------|
| numpy 版本 1.17.x | `conda deactivate && conda activate yopo` |
| rospy 无法导入 | 检查 activate.d hook 脚本是否存在且可执行 |
| roslaunch 找不到 | 确保 `~/.bashrc` 有 `source /opt/ros/noetic/setup.bash` |
| 缺少 ruamel 模块 | `pip install ruamel.yaml==0.17.21` |
