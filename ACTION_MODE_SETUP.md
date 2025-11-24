# Action Mode Setup Guide

## 📋 概述

已实现完整的ROS2 Action模式用于goto_height控制，与现有的Topic模式并行运行，便于对比测试。

## 🔧 编译步骤

### 1. 编译接口包（必须先完成）

```bash
cd /home/robot/Documents/robot_dc/colcon_ws
colcon build --packages-select lift_robot_interfaces
source install/setup.bash
```

### 2. 编译平台节点（包含Action Server）

```bash
colcon build --packages-select lift_robot_platform
source install/setup.bash
```

### 3. 编译Web服务器（包含Action Client）

```bash
colcon build --packages-select lift_robot_web
source install/setup.bash
```

## 🚀 启动测试

### 启动所有节点

```bash
# 终端1: Platform节点（包含Action Server）
ros2 run lift_robot_platform lift_robot_node

# 终端2: Web服务器（包含Action Client）
ros2 run lift_robot_web server

# 终端3: Draw-wire传感器
ros2 run draw_wire_sensor draw_wire_node

# 终端4: 力传感器（可选）
ros2 run lift_robot_force_sensor force_sensor_node
ros2 run lift_robot_force_sensor_2 force_sensor_node_2
```

### 打开Web界面

浏览器访问: `http://localhost:8090`

## 🎯 使用方法

### Web界面布局

1. **原有Topic控制卡片**（绿色边框）
   - 位置：Control Panel顶部
   - 功能：原有的goto_height Topic方式
   - 日志前缀：无（或`[Topic]`）

2. **新增Action控制卡片**（橙色边框）
   - 位置：Control Panel下方（独立卡片）
   - 标题：🚀 Goto Height - Action Mode (Testing)
   - 功能：
     - 左侧：发送Goal、取消Goal
     - 右侧：实时反馈显示（进度条、状态、结果）
   - 日志前缀：`[Action]`

### 操作流程

#### 使用Action Mode：

1. 在Action卡片中输入目标高度（mm）
2. 点击"🚀 Send Action Goal"
3. 观察实时反馈：
   - Goal Status: SENDING → EXECUTING → SUCCEEDED/ABORTED/CANCELLED
   - Progress Bar: 0% → 100%
   - 实时数据: Current Height, Error, Movement State
4. （可选）点击"⏹️ Cancel"中止执行
5. 完成后查看Result信息：
   - Success: True/False
   - Final Height: xxx.xx mm
   - Execution Time: x.xx s
   - Completion Reason: target_reached/cancelled/emergency_stop

#### 对比测试：

可同时使用Topic和Action两种方式，观察：
- 执行效率
- 反馈实时性
- 取消响应速度
- 日志输出差异

## 📊 终端日志区分

### Topic方式日志（原有）
```
Received command: goto_height [SEQ abc123]
[Control] ⬆️ Sending UP command: current=100.00 target=750.00 err=650.00
✅ Target reached: height=750.02mm
```

### Action方式日志（新增）
```
[Action] Received goal request: target_height=750.0mm
[Action] Goal ACCEPTED
[Action] Executing goal: target_height=750.0mm
[Action] Started closed-loop control: target=750.0mm, current=100.00mm
[Action] ✅ Goal SUCCEEDED - final_height=750.02mm, time=5.32s, reason=target_reached
```

## 🔍 验证Action Server是否运行

```bash
# 查看Action列表
ros2 action list

# 应该看到：
# /lift_robot_platform/goto_height

# 查看Action详情
ros2 action info /lift_robot_platform/goto_height

# 发送测试Goal（命令行）
ros2 action send_goal /lift_robot_platform/goto_height lift_robot_interfaces/action/GotoHeight "{target_height: 750.0}"
```

## 📁 已修改的文件

### 新增文件
1. `lift_robot_interfaces/action/GotoHeight.action` - Action接口定义
2. `lift_robot_interfaces/package.xml` - 接口包配置
3. `lift_robot_interfaces/CMakeLists.txt` - 编译配置

### 修改文件
1. `lift_robot_platform/lift_robot_node.py` - 添加Action Server
2. `lift_robot_web/server.py` - 添加Action Client和API端点
3. `lift_robot_web/web/index.html` - 添加Action控制卡片和JavaScript

## ⚠️ 注意事项

1. **编译顺序很重要**：必须先编译`lift_robot_interfaces`，否则其他包无法找到Action定义
2. **source必须执行**：每次编译后必须`source install/setup.bash`
3. **兼容性**：Action Mode和Topic Mode完全独立，互不影响
4. **系统锁**：Action和Topic共享`system_busy`锁，同时只能执行一个
5. **安全特性**：Action Mode继承所有现有安全检查（range limits、force limits、overshoot detection等）

## 🐛 故障排除

### Action Server未启动
```bash
# 检查接口包是否编译
ros2 interface list | grep GotoHeight

# 如果没有输出，重新编译接口包
colcon build --packages-select lift_robot_interfaces --symlink-install
source install/setup.bash
```

### Web界面找不到Action卡片
- 清除浏览器缓存：Ctrl+Shift+R 强制刷新
- 检查index.html是否正确部署

### Action Client连接失败
- 确保Platform节点已启动
- 检查Web服务器日志：`[Action] ⚠️ GotoHeight Action Server not available`
- 等待5秒后Action Server会自动连接

## 📈 性能对比

| 特性 | Topic Mode | Action Mode |
|------|-----------|-------------|
| 反馈频率 | 无（仅状态topic 10Hz） | 10Hz专用Feedback |
| 进度显示 | 无 | 有（0-100%） |
| 取消支持 | 手动stop命令 | 原生cancel机制 |
| 结果报告 | 通过状态topic | 结构化Result消息 |
| 超时检测 | 无 | 可配置超时 |
| 日志标识 | 无 | `[Action]`前缀 |

## 🎉 后续扩展

可以基于此架构继续添加：
- Force控制的Action接口
- Hybrid控制的Action接口  
- Pushrod控制的Action接口
- 多Goal队列执行
- Action超时配置
- 自定义Feedback字段
