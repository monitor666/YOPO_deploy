# YOPO 环境配置详情

## 环境基本信息

| 配置项 | 值 |
|--------|-----|
| conda 发行版 | miniforge3 |
| conda 安装路径 | `/home/amov/miniforge3` |
| yopo 环境路径 | `/home/amov/miniforge3/envs/yopo` |
| Python 版本 | 3.8.20 |
| 硬件平台 | NVIDIA Jetson (aarch64) |

---

## 1. 创建 conda 环境

```bash
conda create -n yopo python=3.8 -y
conda activate yopo
```

---

## 2. 安装 Jetson 专用 PyTorch

**重要**: Jetson 平台必须使用 NVIDIA 提供的专用 PyTorch 版本。

| 包名 | 版本 | 说明 |
|------|------|------|
| torch | 2.0.0a0+8aa34602.nv23.3 | Jetson 专用版本，禁止升级 |
| torchvision | 0.15.1 | 与 torch 配套的 Jetson 版本 |
| torch2trt | 0.5.0 | TensorRT 转换工具 |

### 安装方法

从 NVIDIA 预装或 wheel 文件安装，**禁止**从 PyPI 安装 CUDA 版本。

```bash
# 从 NVIDIA 提供的 wheel 安装
# pip install torch-2.0.0a0+8aa34602.nv23.3-cp38-cp38-linux_aarch64.whl
# pip install torchvision-0.15.1-cp38-cp38-linux_aarch64.whl
```

**注意**: torchaudio 在 Jetson aarch64 平台不兼容，跳过安装。

---

## 3. 安装其他依赖

### 科学计算

```bash
pip install numpy==1.24.4 scipy==1.10.1 scikit-learn==1.3.2
```

### 视觉与点云

```bash
pip install opencv-python-headless==4.12.0.88 open3d==0.18.0 pillow==10.4.0
```

### 配置与工具

```bash
pip install ruamel.yaml==0.17.21 tensorboard==2.14.0 rich scikit-build==0.18.1 empy==4.2
```

### ROS 相关

```bash
pip install catkin_pkg rospkg netifaces
```

### TensorRT 转换

```bash
pip install torch2trt==0.5.0
```

---

## 4. 完整依赖列表

| 类别 | 包名 | 版本 | 说明 |
|------|------|------|------|
| 深度学习 | torch | 2.0.0a0+nv23.3 | Jetson 专用 |
| 深度学习 | torchvision | 0.15.1 | Jetson 专用 |
| 深度学习 | torch2trt | 0.5.0 | TensorRT 转换 |
| 科学计算 | numpy | 1.24.4 | 优先级高于 ROS numpy |
| 科学计算 | scipy | 1.10.1 | - |
| 科学计算 | scikit-learn | 1.3.2 | - |
| 视觉 | opencv-python-headless | 4.12.0.88 | 无 GUI 依赖 |
| 视觉 | open3d | 0.18.0 | 3D 点云处理 |
| 视觉 | pillow | 10.4.0 | 图像处理 |
| 配置 | ruamel.yaml | 0.17.21 | YAML 解析 |
| 工具 | tensorboard | 2.14.0 | 训练可视化 |
| 工具 | rich | 14.3.2 | 终端美化 |
| 构建 | scikit-build | 0.18.1 | CMake 构建 |
| 构建 | empy | 4.2 | 模板处理 |
| ROS | catkin_pkg | latest | ROS 包管理 |
| ROS | rospkg | latest | ROS 包工具 |
| ROS | netifaces | latest | 网络接口 |

---

## 5. 依赖约束

| 约束类型 | 说明 |
|----------|------|
| torch/torchvision | **必须**使用 Jetson 专用版本，禁止从 PyPI 安装 CUDA 版本 |
| torchaudio | Jetson aarch64 平台不兼容，跳过安装 |
| numpy | 保持 1.24.4，高于 ROS 的 1.17.4，低于可能不兼容的 2.x |
| open3d | 使用 0.18.0，0.19.0 可能与 aarch64 不兼容 |

---

## 6. ROS 隔离配置

### 问题背景

conda 和 ROS 存在 PYTHONPATH 冲突：

| 冲突点 | conda (yopo) | ROS (系统) | 影响 |
|--------|--------------|------------|------|
| numpy 版本 | 1.24.4 | 1.17.4 | 神经网络推理出错 |
| Python 路径 | miniforge3/envs/yopo | /usr | 包导入混乱 |
| PYTHONPATH | 需要 conda 优先 | ROS 路径被全局设置 | 导入错误版本的包 |

### 解决方案: conda hooks

使用 conda hooks 机制实现自动环境隔离。

### 目录结构

```
/home/amov/miniforge3/envs/yopo/etc/conda/
├── activate.d/
│   └── yopo_env_setup.sh      # 激活时执行
└── deactivate.d/
    └── yopo_env_cleanup.sh    # 退出时执行
```

### 激活脚本

**文件**: `/home/amov/miniforge3/envs/yopo/etc/conda/activate.d/yopo_env_setup.sh`

```bash
#!/bin/bash
# YOPO Conda 环境激活 Hook

# 保存原始环境变量
export _CONDA_YOPO_OLD_PYTHONPATH="$PYTHONPATH"
export _CONDA_YOPO_OLD_ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH"

# conda 优先的 PYTHONPATH
CONDA_SITE_PACKAGES="$CONDA_PREFIX/lib/python3.8/site-packages"
export PYTHONPATH="$CONDA_SITE_PACKAGES"

# 添加 ROS 路径 (低优先级)
[ -d "/opt/ros/noetic/lib/python3/dist-packages" ] && \
    export PYTHONPATH="$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages"
[ -d "/home/amov/catkin_ws/devel/lib/python3/dist-packages" ] && \
    export PYTHONPATH="$PYTHONPATH:/home/amov/catkin_ws/devel/lib/python3/dist-packages"
[ -d "/home/amov/Projects/YOPO/Controller/devel/lib/python3/dist-packages" ] && \
    export PYTHONPATH="$PYTHONPATH:/home/amov/Projects/YOPO/Controller/devel/lib/python3/dist-packages"

# YOPO 专用 ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH="/home/amov/Projects/YOPO/Controller/src:/home/amov/Projects/YOPO/Simulator/src:${_CONDA_YOPO_OLD_ROS_PACKAGE_PATH}"

# YOPO 项目根目录
export YOPO_ROOT="/home/amov/Projects/YOPO"
```

### 退出脚本

**文件**: `/home/amov/miniforge3/envs/yopo/etc/conda/deactivate.d/yopo_env_cleanup.sh`

```bash
#!/bin/bash
# YOPO Conda 环境退出 Hook

# 恢复原始 PYTHONPATH
[ -n "$_CONDA_YOPO_OLD_PYTHONPATH" ] && export PYTHONPATH="$_CONDA_YOPO_OLD_PYTHONPATH" || unset PYTHONPATH
unset _CONDA_YOPO_OLD_PYTHONPATH

# 恢复原始 ROS_PACKAGE_PATH
[ -n "$_CONDA_YOPO_OLD_ROS_PACKAGE_PATH" ] && export ROS_PACKAGE_PATH="$_CONDA_YOPO_OLD_ROS_PACKAGE_PATH"
unset _CONDA_YOPO_OLD_ROS_PACKAGE_PATH

# 清理 YOPO 变量
unset YOPO_ROOT
```

### 创建脚本命令

```bash
mkdir -p ~/miniforge3/envs/yopo/etc/conda/activate.d
mkdir -p ~/miniforge3/envs/yopo/etc/conda/deactivate.d

# 创建激活脚本 (内容见上方)
# 创建退出脚本 (内容见上方)

chmod +x ~/miniforge3/envs/yopo/etc/conda/activate.d/yopo_env_setup.sh
chmod +x ~/miniforge3/envs/yopo/etc/conda/deactivate.d/yopo_env_cleanup.sh
```

---

## 7. PYTHONPATH 优先级

隔离后的 PYTHONPATH 结构：

```
优先级 1 (最高): /home/amov/miniforge3/envs/yopo/lib/python3.8/site-packages
优先级 2:        /opt/ros/noetic/lib/python3/dist-packages
优先级 3:        /home/amov/catkin_ws/devel/lib/python3/dist-packages
优先级 4 (最低): /home/amov/Projects/YOPO/Controller/devel/lib/python3/dist-packages
```

### 包加载来源

| 包 | 来源 | 版本 |
|----|------|------|
| numpy | conda (优先级 1) | 1.24.4 |
| torch | conda (优先级 1) | 2.0.0a0+nv23.3 |
| opencv | conda (优先级 1) | 4.12.0 |
| rospy | ROS (优先级 2) | noetic |
| cv_bridge | ROS (优先级 2) | noetic |
| sensor_msgs | ROS (优先级 2) | noetic |

---

## 8. 验证方法

### 验证 numpy 来源

```bash
conda activate yopo
python -c "import numpy; print(numpy.__version__, numpy.__file__)"
```

期望输出：
```
1.24.4 /home/amov/miniforge3/envs/yopo/lib/python3.8/site-packages/numpy/__init__.py
```

### 验证 ROS 可用性

```bash
conda activate yopo
python -c "import rospy; print('rospy OK')"
rostopic list  # 需要 roscore 运行
```

### 完整依赖检查

```bash
conda activate yopo
python -c "
import numpy, torch, cv2, rospy, open3d
print(f'numpy {numpy.__version__}: {numpy.__file__}')
print(f'torch {torch.__version__}')
print(f'cv2 {cv2.__version__}')
print(f'open3d {open3d.__version__}')
print('rospy OK')
"
```

---

## 9. 故障排查

### numpy 版本错误

**症状**: `numpy.__version__` 显示 1.17.x

**解决**: 
```bash
conda deactivate
conda activate yopo  # 重新激活触发 hooks
```

### rospy 无法导入

**症状**: `ModuleNotFoundError: No module named 'rospy'`

**解决**: 检查 activate.d hook 脚本是否存在且可执行

### roslaunch 命令不可用

**症状**: `command not found: roslaunch`

**解决**: 确保 `.bashrc` 中有 `source /opt/ros/noetic/setup.bash`

### conda hooks 脚本丢失

**症状**: 激活 yopo 后 PYTHONPATH 不正确

**解决**: 参考"创建脚本命令"章节重建脚本

### 缺少 ruamel 模块

**症状**: `ModuleNotFoundError: No module named 'ruamel'`

**解决**:
```bash
conda activate yopo
pip install ruamel.yaml==0.17.21
```
