# Manual Down 任务自动完成检测

## 功能说明

当执行 `down` 命令（手动下降）时，平台会自动检测高度，当高度低于 **832mm** 时：

1. ✅ 将任务状态标记为 `completed`
2. ✅ 设置完成原因为 `target_reached`
3. ✅ 更新 `movement_state` 为 `stop`
4. ❌ **不发送停止指令**（硬件会自动停止）

## 实现位置

**文件**: `lift_robot_platform/lift_robot_node.py`

**位置**: `control_loop()` 方法中，软件限位检测之后，力控检测之前

```python
# Manual down task: check if reached bottom (height < 832mm)
if self.task_state == 'running' and self.task_type == 'manual_down':
    if self.current_height < 832.0:
        # Reached bottom, mark as completed (don't send stop, hardware auto-stops)
        self._complete_task('target_reached')
        self.movement_state = 'stop'  # Update state to reflect hardware stop
        self.get_logger().info(
            f"[ManualDown] Bottom reached: height={self.current_height:.2f}mm < 832mm (hardware auto-stop)"
        )
```

## 使用场景

### 1. Web 界面点击 Down 按钮
```
1. 用户点击 "Down" 按钮
2. Platform 开始下降，task_state = 'running', task_type = 'manual_down'
3. 高度传感器实时监控（50Hz）
4. 当 height < 832mm:
   - task_state → 'completed'
   - completion_reason → 'target_reached'
   - movement_state → 'stop'
   - 日志: "[ManualDown] Bottom reached: height=831.45mm < 832mm (hardware auto-stop)"
```

### 2. HTTP API 发送 down 命令
```python
import requests

# 发送 down 命令
requests.post('http://192.168.1.3:8090/api/cmd', json={
    'command': 'down',
    'target': 'platform'
})

# 监控状态
while True:
    status = requests.get('http://192.168.1.3:8090/api/status').json()
    platform = status['platform']
    
    print(f"Height: {platform['current_height']:.2f}mm")
    print(f"Task State: {platform['task_state']}")
    
    if platform['task_state'] == 'completed':
        print(f"✅ Reached bottom: {platform['completion_reason']}")
        break
    
    time.sleep(0.2)
```

### 3. ROS2 Topic 发送命令
```bash
ros2 topic pub --once /lift_robot_platform/command std_msgs/String \
  'data: "{\"command\": \"down\"}"'

# 监控状态
ros2 topic echo /lift_robot_platform/status
```

## 为什么不发送停止指令

**原因**: 硬件限位开关会在平台到达底部时**自动断开电源**，无需软件发送停止指令。

**好处**:
1. 避免与硬件限位冲突
2. 减少不必要的 Modbus 通信
3. 保持与硬件行为一致

## 状态监控

### Web 界面显示
```
Task Status
├─ State: RUNNING → COMPLETED (绿色)
├─ Task Type: manual_down
├─ Duration: 8.5s
└─ Completion: target_reached
```

### ROS2 日志输出
```
[INFO] [lift_robot_platform]: [ManualDown] Bottom reached: height=831.23mm < 832mm (hardware auto-stop)
```

### HTTP API 状态
```json
{
  "platform": {
    "task_state": "completed",
    "task_type": "manual_down",
    "task_duration": 8.52,
    "completion_reason": "target_reached",
    "current_height": 831.23,
    "movement_state": "stop"
  }
}
```

## 检测参数

| 参数 | 值 | 说明 |
|------|-----|------|
| **检测高度** | 832.0 mm | 低于此高度认为到达底部 |
| **检测频率** | 50 Hz | 控制循环频率（每 20ms 检测一次） |
| **任务类型** | `manual_down` | 仅对手动下降任务生效 |
| **完成原因** | `target_reached` | 标记为目标到达 |

## 注意事项

1. **仅对 `manual_down` 生效**: 
   - `goto_height` 任务有自己的目标检测逻辑
   - `force_down` 任务有力控停止逻辑
   
2. **硬件优先**: 
   - 软件检测仅用于状态更新
   - 实际停止由硬件限位控制
   
3. **高度阈值**: 
   - 832mm 是底部检测阈值
   - 可根据实际硬件限位位置调整

4. **软件限位**: 
   - 上限软件限位仍为 950mm（保持不变）
   - 下限由硬件限位控制，软件仅监控

## 相关代码

- **任务开始**: `command_callback()` - 处理 `down` 命令时调用 `_start_task('manual_down')`
- **任务完成**: `control_loop()` - 检测高度并调用 `_complete_task('target_reached')`
- **状态发布**: `publish_status()` - 包含 task_state, task_type, completion_reason

## 测试建议

```bash
# 1. 启动系统
ros2 launch robot_bringup lift_robot_bringup.py

# 2. 打开 Web 界面
http://192.168.1.3:8090

# 3. 点击 "Down" 按钮

# 4. 观察:
# - Task Status 卡片从 RUNNING → COMPLETED
# - 高度数值低于 832mm
# - Completion 显示 "target_reached"

# 5. 检查日志
# ros2 topic echo /lift_robot_platform/status
```
