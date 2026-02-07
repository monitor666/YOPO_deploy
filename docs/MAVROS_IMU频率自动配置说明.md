# MAVROS IMU 频率自动配置说明

## 问题背景

### 现象
- MAVROS 默认 IMU 数据频率只有 ~50Hz
- VINS-Fusion 和 YOPO SO3 控制器需要 200Hz 以上的 IMU 数据
- 原来需要每次启动后手动执行 `rosservice call` 设置频率

### 根本原因
1. **MAVLink 数据流频率按通道独立配置**：PX4 飞控对不同连接（USB/UART/UDP）有独立的频率设置
2. **MAVROS 默认请求频率较低**：约 50Hz，不满足 VIO/控制需求
3. **本地 mavros_msgs 缺少服务定义**：YOPO/Controller 工作空间的精简版 mavros_msgs 缺少 `MessageInterval.srv`，当 source 该环境时会覆盖系统版本，导致无法调用 rosservice

### MAVLink 消息 ID 参考
| 消息名称 | ID | 说明 |
|---------|-----|------|
| HIGHRES_IMU | 105 | 高分辨率 IMU 原始数据 |
| ATTITUDE_QUATERNION | 31 | 姿态四元数 |
| SCALED_IMU | 26 | 缩放后的 IMU 数据 |
| RAW_IMU | 27 | 原始 IMU 数据 |

---

## 解决方案概述

通过以下修改实现 **启动时自动设置 IMU 频率为 200Hz**：

1. 补全本地 mavros_msgs 的 `MessageInterval.srv` 定义
2. 创建 `yopo_tools` 工具包，包含自动配置脚本
3. 修改 `px4_yopo.launch` 自动启动配置节点

---

## 修改详情

### 修改 1：补全 mavros_msgs 服务定义

**目的**：解决 YOPO/Controller 环境下（或同时 source 多个工作空间时）`rosservice call` 报错 `Unable to load type [mavros_msgs/MessageInterval]`

> **说明**：YOPO/Controller 工作空间包含一个精简版 mavros_msgs，当 source 该环境时会覆盖系统完整版，导致缺少 MessageInterval.srv

#### 新建文件
**路径**：`YOPO/Controller/src/utils/mavros_msgs/srv/MessageInterval.srv`

```srv
# sets message interval
# See MAV_CMD_SET_MESSAGE_INTERVAL

uint32 message_id
float32 message_rate
---
bool success
```

#### 修改文件
**路径**：`YOPO/Controller/src/utils/mavros_msgs/CMakeLists.txt`

**修改内容**：在 `add_service_files` 中添加 `MessageInterval.srv`

```cmake
add_service_files(
  DIRECTORY srv
  FILES
  CommandBool.srv
  MessageInterval.srv   # 新增此行
  SetMode.srv
)
```

---

### 修改 2：创建 yopo_tools 工具包

**目的**：提供可在 launch 文件中调用的 IMU 频率配置节点

#### 目录结构
```
catkin_ws/src/yopo_tools/
├── package.xml
├── CMakeLists.txt
└── scripts/
    └── set_mavros_imu_rate.py
```

#### package.xml
**路径**：`catkin_ws/src/yopo_tools/package.xml`

```xml
<?xml version="1.0"?>
<package format="2">
  <name>yopo_tools</name>
  <version>1.0.0</version>
  <description>YOPO 系统实用工具包</description>

  <maintainer email="user@example.com">amov</maintainer>
  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>
  
  <build_depend>rospy</build_depend>
  <build_depend>mavros_msgs</build_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>mavros_msgs</exec_depend>
</package>
```

#### CMakeLists.txt
**路径**：`catkin_ws/src/yopo_tools/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(yopo_tools)

find_package(catkin REQUIRED COMPONENTS rospy mavros_msgs)

catkin_package()

# 安装 Python 脚本
catkin_install_python(PROGRAMS
  scripts/set_mavros_imu_rate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

#### set_mavros_imu_rate.py
**路径**：`catkin_ws/src/yopo_tools/scripts/set_mavros_imu_rate.py`

**功能**：
- 订阅 `/mavros/state` 监控飞控连接状态
- 检测到连接后延迟 2 秒，等待服务就绪
- 调用 `/mavros/set_message_interval` 设置 IMU 频率
- 持续运行，飞控断开重连时自动重新配置

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
MAVROS IMU 频率设置脚本

功能: 监控 MAVROS 连接状态，自动设置 IMU 数据流为指定频率
      当飞控断开重连时，自动重新设置频率

MAVLink 消息 ID:
  - HIGHRES_IMU (105): 高分辨率 IMU 原始数据
  - ATTITUDE_QUATERNION (31): 姿态四元数
"""

import rospy
from mavros_msgs.srv import MessageInterval
from mavros_msgs.msg import State

class ImuRateSetter:
    def __init__(self):
        rospy.init_node('set_imu_rate', anonymous=True)
        
        # 获取 IMU 频率参数，默认 200Hz
        self.imu_rate = rospy.get_param('~imu_rate', 200.0)
        
        # 连接状态跟踪
        self.connected = False
        self.rate_configured = False
        
        # 订阅飞控连接状态
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        
        rospy.loginfo("[IMU Rate] 节点已启动，目标频率: %.0fHz", self.imu_rate)
        rospy.loginfo("[IMU Rate] 等待飞控连接...")
    
    def state_callback(self, msg):
        """监控飞控连接状态，连接后设置 IMU 频率"""
        if msg.connected and not self.connected:
            rospy.loginfo("[IMU Rate] 检测到飞控连接，等待服务就绪...")
            self.connected = True
            self.rate_configured = False
            rospy.Timer(rospy.Duration(2.0), self.configure_rate, oneshot=True)
        elif not msg.connected and self.connected:
            rospy.logwarn("[IMU Rate] 飞控断开连接，等待重连...")
            self.connected = False
            self.rate_configured = False
    
    def configure_rate(self, event=None):
        """配置 IMU 数据流频率"""
        if not self.connected or self.rate_configured:
            return
        
        rospy.loginfo("[IMU Rate] 正在配置 IMU 频率为 %.0fHz...", self.imu_rate)
        
        success_count = 0
        
        if self.set_message_interval(105, self.imu_rate):
            rospy.loginfo("[IMU Rate] ✓ HIGHRES_IMU (105) 设置为 %.0fHz", self.imu_rate)
            success_count += 1
        else:
            rospy.logwarn("[IMU Rate] ✗ HIGHRES_IMU (105) 设置失败")
        
        if self.set_message_interval(31, self.imu_rate):
            rospy.loginfo("[IMU Rate] ✓ ATTITUDE_QUATERNION (31) 设置为 %.0fHz", self.imu_rate)
            success_count += 1
        else:
            rospy.logwarn("[IMU Rate] ✗ ATTITUDE_QUATERNION (31) 设置失败")
        
        if success_count == 2:
            rospy.loginfo("[IMU Rate] 配置完成！可使用 'rostopic hz /mavros/imu/data_raw' 验证")
            self.rate_configured = True
    
    def set_message_interval(self, message_id, rate):
        """设置 MAVLink 消息发送间隔"""
        service_name = '/mavros/set_message_interval'
        try:
            rospy.wait_for_service(service_name, timeout=10.0)
            set_interval = rospy.ServiceProxy(service_name, MessageInterval)
            resp = set_interval(message_id=message_id, message_rate=rate)
            return resp.success
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr("[IMU Rate] 服务调用失败: %s", str(e))
            return False
    
    def run(self):
        """保持节点运行，持续监控连接状态"""
        rospy.spin()

if __name__ == '__main__':
    try:
        setter = ImuRateSetter()
        setter.run()
    except rospy.ROSInterruptException:
        pass
```

---

### 修改 3：更新 px4_yopo.launch

**目的**：启动 MAVROS 时自动运行 IMU 频率配置节点

**路径**：`configs/mavros/px4_yopo.launch`

**新增内容**（在 `</launch>` 前添加）：

```xml
<!-- ================================================================== -->
<!-- 自动设置 IMU 数据流频率                                           -->
<!-- ================================================================== -->
<!-- 
    启动后自动调用脚本设置 IMU 频率为 $(arg imu_rate) Hz
    脚本会等待 MAVROS 服务就绪后再执行
-->
<node name="set_imu_rate" pkg="yopo_tools" type="set_mavros_imu_rate.py" output="screen">
    <param name="imu_rate" value="$(arg imu_rate)"/>
</node>
```

---

## 工作流程

```
roslaunch px4_yopo.launch
         │
         ├──▶ mavros_node 启动，连接飞控
         │
         └──▶ set_mavros_imu_rate.py 启动
                    │
                    ├── 订阅 /mavros/state
                    │
                    ▼
              检测到飞控连接 (connected=True)
                    │
                    ├── 等待 2 秒（确保服务就绪）
                    │
                    ▼
              调用 /mavros/set_message_interval 服务
                    │
                    ├── 设置 HIGHRES_IMU (105) → 200Hz
                    └── 设置 ATTITUDE_QUATERNION (31) → 200Hz
                    │
                    ▼
              持续监控，断开重连时自动重设
```

---

## 使用方法

### 首次使用（需编译）

```bash
cd ~/Projects/catkin_ws
catkin_make
```

### 日常使用

```bash
source ~/Projects/catkin_ws/devel/setup.bash
roslaunch ~/Projects/configs/mavros/px4_yopo.launch fcu_url:=/dev/ttyACM0:921600
```

### 自定义 IMU 频率

```bash
roslaunch ~/Projects/configs/mavros/px4_yopo.launch fcu_url:=/dev/ttyACM0:921600 imu_rate:=250
```

### 验证

```bash
rostopic hz /mavros/imu/data_raw
# 应显示 ~200Hz
```

---

## 相关文件清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `YOPO/Controller/src/utils/mavros_msgs/srv/MessageInterval.srv` | 新建 | 服务定义 |
| `YOPO/Controller/src/utils/mavros_msgs/CMakeLists.txt` | 修改 | 添加服务文件 |
| `catkin_ws/src/yopo_tools/package.xml` | 新建 | 包描述 |
| `catkin_ws/src/yopo_tools/CMakeLists.txt` | 新建 | 编译配置 |
| `catkin_ws/src/yopo_tools/scripts/set_mavros_imu_rate.py` | 新建 | 配置脚本 |
| `configs/mavros/px4_yopo.launch` | 修改 | 添加节点 |

---

## 故障排查

### IMU 频率仍为 50Hz

1. 检查是否重新编译了 catkin_ws
2. 检查是否 source 了正确的环境：`source ~/Projects/catkin_ws/devel/setup.bash`
3. 查看 launch 输出是否有 `[IMU Rate] ✓` 成功日志

### 报错 `Unable to load type [mavros_msgs/MessageInterval]`

1. 确认 `MessageInterval.srv` 已添加到 mavros_msgs
2. 确认 `CMakeLists.txt` 已更新
3. 重新编译：`cd ~/Projects/catkin_ws && catkin_make`

### 配置脚本未启动

1. 检查 launch 文件是否包含 `set_imu_rate` 节点
2. 检查 yopo_tools 包是否正确编译：`rospack find yopo_tools`

---

## 修改日期

- 2026-02-07: 初始实现
