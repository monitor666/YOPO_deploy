# YOPO 虚拟环境与隔离配置说明

> **文档用途**: 记录 YOPO 项目的 conda 虚拟环境配置和 ROS 隔离配置。AI 助手在处理 YOPO 环境搭建、依赖安装、环境隔离等问题时，应参考本文档。

---

## 元信息

| 属性 | 值 |
|------|-----|
| 项目名称 | YOPO (You Only Plan Once) |
| 文档类型 | 虚拟环境与隔离配置说明 |
| conda 发行版 | miniforge3 |
| conda 环境名 | `yopo` |
| Python 版本 | 3.8.20 |
| 硬件平台 | NVIDIA Jetson (aarch64) |
| 创建日期 | 2026-02-03 |
| 最后更新 | 2026-02-03 |

---

## 第一部分：YOPO 虚拟环境配置

### 环境基本信息

| 配置项 | 值 |
|--------|-----|
| conda 安装路径 | `/home/amov/miniforge3` |
| yopo 环境路径 | `/home/amov/miniforge3/envs/yopo` |
| Python 解释器 | `/home/amov/miniforge3/envs/yopo/bin/python` |
| site-packages | `/home/amov/miniforge3/envs/yopo/lib/python3.8/site-packages` |

### 核心依赖包

#### 深度学习框架 (Jetson 专用版本)

| 包名 | 版本 | 说明 |
|------|------|------|
| torch | 2.0.0a0+8aa34602.nv23.3 | NVIDIA Jetson 专用版本，勿升级 |
| torchvision | 0.15.1 | 与 torch 配套的 Jetson 版本 |
| torch2trt | 0.5.0 | TensorRT 转换工具 |

#### 科学计算

| 包名 | 版本 | 说明 |
|------|------|------|
| numpy | 1.24.4 | 与 torch/ROS 兼容，优先级高于 ROS numpy |
| scipy | 1.10.1 | 科学计算 |
| scikit-learn | 1.3.2 | 机器学习 |

#### 视觉与点云

| 包名 | 版本 | 说明 |
|------|------|------|
| opencv-python-headless | 4.12.0.88 | 无 GUI 依赖的 OpenCV |
| open3d | 0.18.0 | 3D 点云处理 |
| pillow | 10.4.0 | 图像处理 |

#### 配置与工具

| 包名 | 版本 | 说明 |
|------|------|------|
| ruamel.yaml | 0.17.21 | YAML 配置解析 |
| tensorboard | 2.14.0 | 训练可视化 |
| rich | 14.3.2 | 终端美化输出 |
| empy | 4.2 | 模板处理 (ROS 相关) |
| scikit-build | 0.18.1 | CMake 构建工具 |

### 环境创建/恢复命令

如需重建 yopo 环境，执行以下命令：

```bash
# 创建基础环境
conda create -n yopo python=3.8 -y

# 激活环境
conda activate yopo

# 安装 Jetson 专用 PyTorch (从 NVIDIA 预装或 wheel 安装)
# 注意: torch/torchvision 应使用 Jetson 专用版本，不要从 PyPI 安装

# 安装其他依赖
pip install numpy==1.24.4 scipy==1.10.1 scikit-learn==1.3.2
pip install opencv-python-headless==4.12.0.88 open3d==0.18.0 pillow==10.4.0
pip install ruamel.yaml==0.17.21 tensorboard==2.14.0 rich scikit-build==0.18.1 empy==4.2
pip install catkin_pkg rospkg netifaces
```

### 依赖约束

| 约束类型 | 说明 |
|----------|------|
| torch/torchvision | **必须**使用 Jetson 专用版本，禁止从 PyPI 安装 CUDA 版本 |
| torchaudio | Jetson aarch64 平台不兼容，跳过安装 |
| numpy | 保持 1.24.4，高于 ROS 的 1.17.4，低于可能不兼容的 2.x |
| open3d | 使用 0.18.0，0.19.0 可能与 aarch64 不兼容 |

---

## 第二部分：ROS 环境隔离配置

### 问题背景

#### 原始冲突

YOPO 项目需要同时使用 **conda 虚拟环境** 和 **ROS Noetic**，但两者存在依赖冲突：

| 冲突点 | conda (yopo) | ROS (系统) | 影响 |
|--------|--------------|------------|------|
| numpy 版本 | 1.24.4 | 1.17.4 | 神经网络推理出错 |
| Python 路径 | miniforge3/envs/yopo | /usr | 包导入混乱 |
| PYTHONPATH | 需要 conda 优先 | ROS 路径被全局设置 | 导入错误版本的包 |

#### 问题表现

当直接使用 `conda activate yopo` 时，由于 `.bashrc` 中已 source 了 ROS 环境，`PYTHONPATH` 包含 ROS 路径且优先级高于 conda，导致：

```python
import numpy  # 可能导入 ROS 的 1.17.4 而非 conda 的 1.24.4
```

### 解决方案

#### 方案选择

使用 **conda hooks 机制** 实现自动环境隔离：

- `activate.d/` - 环境激活时自动执行的脚本
- `deactivate.d/` - 环境退出时自动执行的脚本

#### 优势

| 特性 | 说明 |
|------|------|
| 自动化 | 无需手动执行额外脚本 |
| 透明性 | 使用标准 conda 命令即可 |
| 可维护性 | 配置与环境绑定，不会丢失 |
| 可恢复性 | 退出时自动恢复原始环境 |

---

### 配置文件

#### 文件位置

```
/home/amov/miniforge3/envs/yopo/etc/conda/
├── activate.d/
│   └── yopo_env_setup.sh      # 激活时执行
└── deactivate.d/
    └── yopo_env_cleanup.sh    # 退出时执行
```

#### 激活脚本内容

**文件**: `/home/amov/miniforge3/envs/yopo/etc/conda/activate.d/yopo_env_setup.sh`

```bash
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

# 设置 YOPO 专用的 ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH="/home/amov/Projects/YOPO/Controller/src:/home/amov/Projects/YOPO/Simulator/src:${_CONDA_YOPO_OLD_ROS_PACKAGE_PATH}"

# 设置 YOPO 项目根目录
export YOPO_ROOT="/home/amov/Projects/YOPO"
```

#### 退出脚本内容

**文件**: `/home/amov/miniforge3/envs/yopo/etc/conda/deactivate.d/yopo_env_cleanup.sh`

```bash
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
```

#### 隔离脚本恢复命令

如果 conda hooks 脚本丢失（如重装 miniforge3 后），执行以下命令恢复：

```bash
# 创建目录
mkdir -p ~/miniforge3/envs/yopo/etc/conda/activate.d
mkdir -p ~/miniforge3/envs/yopo/etc/conda/deactivate.d

# 创建激活脚本 (内容见上方)
# 创建退出脚本 (内容见上方)

# 设置执行权限
chmod +x ~/miniforge3/envs/yopo/etc/conda/activate.d/yopo_env_setup.sh
chmod +x ~/miniforge3/envs/yopo/etc/conda/deactivate.d/yopo_env_cleanup.sh
```

---

### PYTHONPATH 优先级

隔离后的 PYTHONPATH 结构：

```
优先级 1 (最高): /home/amov/miniforge3/envs/yopo/lib/python3.8/site-packages
优先级 2:        /opt/ros/noetic/lib/python3/dist-packages
优先级 3:        /home/amov/catkin_ws/devel/lib/python3/dist-packages
优先级 4 (最低): /home/amov/Projects/YOPO/Controller/devel/lib/python3/dist-packages
```

#### 包加载来源

| 包 | 来源 | 版本 |
|----|------|------|
| numpy | conda (优先级 1) | 1.24.4 |
| torch | conda (优先级 1) | 2.0.0a0+nv23.3 |
| opencv | conda (优先级 1) | 4.12.0 |
| rospy | ROS (优先级 2) | noetic |
| cv_bridge | ROS (优先级 2) | noetic |
| sensor_msgs | ROS (优先级 2) | noetic |

---

## 第三部分：使用方法

### 激活环境

```bash
conda activate yopo
```

自动执行隔离，PYTHONPATH 自动配置为 conda 优先。

### 退出环境

```bash
conda deactivate
```

自动恢复原始环境变量。

### 运行 YOPO

```bash
conda activate yopo
cd /home/amov/Projects/YOPO/YOPO
python test_yopo_ros.py --trial=1 --epoch=50
```

---

## 第四部分：验证方法

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

## 第五部分：约束条件

### 强制约束 (MUST)

| ID | 约束内容 |
|----|----------|
| E-001 | **必须**使用 `conda activate yopo` 激活环境，不要直接修改 PYTHONPATH |
| E-002 | **禁止**在 `.bashrc` 中添加 YOPO 专用的环境变量（应放在 conda hooks 中） |
| E-003 | **禁止**手动修改 yopo 环境的 PYTHONPATH 覆盖 hooks 的设置 |
| E-004 | **禁止**从 PyPI 安装 CUDA 版本的 torch/torchvision，必须使用 Jetson 专用版本 |

### 建议约束 (SHOULD)

| ID | 约束内容 |
|----|----------|
| E-101 | 运行 YOPO 前应验证 numpy 来自 conda 环境 |
| E-102 | 如需修改隔离配置，应编辑 conda hooks 脚本而非创建新的隔离脚本 |
| E-103 | 退出 YOPO 工作后应执行 `conda deactivate` 恢复环境 |
| E-104 | 安装新依赖前应检查是否与 ROS 存在版本冲突 |

---

## 第六部分：故障排查

### 问题: numpy 版本错误

**症状**: `numpy.__version__` 显示 1.17.x

**原因**: PYTHONPATH 未正确设置

**解决**:
```bash
conda deactivate
conda activate yopo  # 重新激活触发 hooks
```

### 问题: rospy 无法导入

**症状**: `ModuleNotFoundError: No module named 'rospy'`

**原因**: ROS 路径未添加到 PYTHONPATH

**解决**: 检查 `/home/amov/miniforge3/envs/yopo/etc/conda/activate.d/yopo_env_setup.sh` 是否存在且可执行

### 问题: roslaunch 命令不可用

**症状**: `command not found: roslaunch`

**原因**: PATH 中缺少 ROS bin 目录

**解决**: 确保 `.bashrc` 中有 `source /opt/ros/noetic/setup.bash`

### 问题: conda hooks 脚本丢失

**症状**: 激活 yopo 后 PYTHONPATH 不正确

**原因**: 重装 miniforge3 后 hooks 脚本丢失

**解决**: 参考本文档"隔离脚本恢复命令"章节重建脚本

### 问题: ModuleNotFoundError: No module named 'ruamel'

**症状**: 运行 YOPO 时报错缺少 ruamel 模块

**原因**: yopo 环境依赖未完全安装

**解决**:
```bash
conda activate yopo
pip install ruamel.yaml==0.17.21
```

---

## 相关文件

| 文件 | 用途 |
|------|------|
| `/home/amov/.bashrc` | 全局 shell 配置（包含 ROS 基础环境） |
| `/home/amov/miniforge3/envs/yopo/etc/conda/activate.d/yopo_env_setup.sh` | 激活时隔离脚本 |
| `/home/amov/miniforge3/envs/yopo/etc/conda/deactivate.d/yopo_env_cleanup.sh` | 退出时恢复脚本 |
| `/home/amov/Projects/YOPO/YOPO/requirements.txt` | YOPO 原始依赖列表 (参考用) |
| `/home/amov/Projects/YOPO_硬件通信节点说明.md` | 硬件通信接口规范 |

---

## 历史变更

| 日期 | 变更内容 |
|------|----------|
| 2026-02-03 | 初始创建，使用 conda hooks 实现自动隔离 |
| 2026-02-03 | 删除独立隔离脚本 (`/home/amov/Projects/yopo_scripts/`)，迁移至 conda hooks |
| 2026-02-03 | 添加虚拟环境配置说明，补充依赖包列表和恢复命令 |
