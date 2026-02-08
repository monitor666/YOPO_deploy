# YOPO 深度图分辨率自动转换说明

## 概述

本文档说明 YOPO 规划器如何处理深度图分辨率转换。

**结论：无需手动修改代码，现有逻辑已支持自动 resize。**

---

## TensorRT 模型输入规格

通过分析 `yopo_trt_transfer.py` 第 42 行确认：

```python
depth = np.zeros(shape=[1, 1, 96, 160], dtype=np.float32)
```

| 参数 | 值 |
|------|-----|
| 高度 (height) | **96** |
| 宽度 (width) | **160** |
| 格式 | `[batch, channel, height, width]` |

与配置文件 `config/traj_opt.yaml` 一致：

```yaml
image_height: 96
image_width: 160
```

---

## 数据流

```
相机输出 (640×480, 16UC1 毫米单位)
       ↓
深度单位转换 (毫米 → 米)
       ↓
cv2.resize (INTER_NEAREST)
       ↓
神经网络输入 (96×160)
       ↓
TensorRT 推理
       ↓
轨迹规划输出
```

---

## 现有代码分析

`test_yopo_ros.py` 第 158-159 行**已有自动 resize 逻辑**：

```python
if depth.shape[0] != self.height or depth.shape[1] != self.width:
    depth = cv2.resize(depth, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
```

其中：
- `self.height = cfg['image_height']` = 96
- `self.width = cfg['image_width']` = 160

**因此，640×480 的深度图会自动 resize 到 96×160，无需任何代码修改。**

---

## 验证方法

### 1. 确认配置文件

```bash
# 检查 YOPO 配置
cat ~/Projects/YOPO/YOPO/config/traj_opt.yaml | grep -E "image_height|image_width"

# 应输出：
# image_height: 96
# image_width: 160
```

### 2. 确认相机输出

```bash
# 启动相机
roslaunch ~/Projects/configs/realsense/yopo_d455f_camera.launch

# 检查深度图分辨率
rostopic echo /camera/depth/image_rect_raw --noarr | head -20

# 应显示：
# width: 640
# height: 480
# encoding: "16UC1"
```

### 3. 运行 YOPO 规划器

```bash
conda activate yopo
cd ~/Projects/YOPO/YOPO
python test_yopo_ros.py --use_tensorrt=1

# 如果无报错且正常输出控制指令，说明 resize 工作正常
rostopic hz /so3_control/pos_cmd
```

---

## 常见问题

### Q: 需要修改 `traj_opt.yaml` 吗？

A: **不需要**。保持默认的 96×160 即可。这个尺寸是训练时确定的，必须与模型匹配。

### Q: Resize 会影响避障效果吗？

A: 从 640×480 降采样到 96×160 会损失一些细节，但：
1. YOPO 训练时就是用这个分辨率
2. 使用 `INTER_NEAREST` 插值保持深度值准确性
3. 对于避障任务，这个分辨率已经足够

### Q: 如果我想用更高分辨率怎么办？

A: 需要重新训练模型并重新转换 TensorRT。步骤：
1. 修改 `traj_opt.yaml` 中的 `image_height` 和 `image_width`
2. 重新训练：`python train_yopo.py`
3. 重新转换：`python yopo_trt_transfer.py`

---

## 相关文件

| 文件 | 说明 |
|------|------|
| `YOPO/YOPO/test_yopo_ros.py` | ROS 接口，包含自动 resize 逻辑 |
| `YOPO/YOPO/config/traj_opt.yaml` | 配置文件，定义 image_height/width |
| `YOPO/YOPO/yopo_trt_transfer.py` | TensorRT 转换脚本，确认输入尺寸 |
| `configs/realsense/yopo_d455f_camera.launch` | 相机启动文件 |

---

## 版本信息

- **创建日期**: 2026-02-08
- **更新日期**: 2026-02-08
- **更新内容**: 修正分辨率为 96×160
