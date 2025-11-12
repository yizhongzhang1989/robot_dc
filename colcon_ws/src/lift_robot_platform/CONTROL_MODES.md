# Lift Robot Platform Control Modes

## 运动模式互斥规则

### Platform控制模式（互斥）

Platform有**两种控制模式**，同一时间**只能启用一种**：

| 模式 | 使能标志 | 命令 | 目标 | 优先级 |
|------|---------|------|------|--------|
| **高度自动控制** | `control_enabled = True` | `goto_height` | 指定高度(mm) | 互斥 |
| **力控制** | `force_control_active = True` | `force_up`/`force_down` | 目标力(N) | 互斥 |

**互斥保证**：
1. **启动时检查**：新控制模式启动时，自动禁用另一个模式
2. **运行时检查**：control loop每个cycle检查，如发现多个模式同时启用，立即emergency stop

### Pushrod控制（独立）

Pushrod由独立的 `pushrod_node.py` 控制，与Platform控制**不冲突**：
- **高度自动控制**：`goto_height` 命令
- **定时运动**：`timed_up`/`timed_down` 命令

## Reset命令执行流程

Reset是**最高优先级**命令，执行线程安全的系统复位。

### Reset触发方式

Reset可以通过以下方式触发：
1. **HTTP API调用**：通过Web服务器 `POST /api/cmd {"command": "reset", "target": "platform"}`
2. **Node内部触发**：当某个node检测到reset条件时（如安全检测失败等）

### Reset执行步骤（7步流程）

```
Step 1: 设置 reset_in_progress = True
        ↓ (通知control loop停止)
        
Step 2: 等待20ms（1个control cycle）
        ↓ (确保control loop完成当前cycle并退出)
        
Step 3: 禁用所有控制模式
        - control_enabled = False
        - force_control_active = False
        - movement_state = 'stop'
        ↓
        
Step 3.5: 取消所有活动Timer
        ↓ (防止延迟的继电器操作)
        
Step 4: 重置所有继电器为 0（清零所有继电器状态）
        - Platform relays (0, 1, 2) → OFF
        - Pushrod relays (3, 4, 5) → OFF
        ↓ (所有继电器关闭，但不触发停止动作)
        
Step 5: 发送 STOP 脉冲（relay 0 脉冲）
        ↓ (触发硬件物理停止动作)
        
Step 6: 标记任务状态并清除reset flag
        - task_state → 'emergency_stop'
        - 手动reset: reason → 'manual_stop'
        - 安全超限reset: reason → 具体原因（如'force_overshoot'）
        - 释放 system_busy 锁
        - reset_in_progress = False
        - 系统就绪
```

**关键时序保证**：
- **Reset检查位置**：control loop的**最开头**（PRIORITY 0），在所有控制逻辑之前
- **Step 1**：Reset flag设置后，通知control loop停止
- **Step 2**：等待20ms (1个cycle) 确保control loop完成当前cycle并退出，不会再发送新命令
- **Step 3**：禁用所有控制模式标志
- **Step 3.5**：**取消所有活动Timer**（关键！防止延迟的继电器操作）
- **Step 4**：**先清零所有继电器**（让所有relay都变为OFF状态）
- **Step 5**：**再发送stop脉冲**（relay 0 flash触发硬件停止动作）
  - 注意：reset_all_relays只是让继电器OFF，不会触发停止
  - 硬件需要relay 0的脉冲（ON→OFF）才会真正停止运动
- **Step 6**：标记任务状态并释放锁
- **双节点协调**：Web服务器同时向Platform和Pushrod发送reset命令，确保完整系统复位

### Emergency Stop自动触发Reset

当检测到以下安全超限情况时，系统会**自动触发reset流程**（而非简单发送stop命令）：

| 触发条件 | 阈值 | Emergency Reason |
|---------|------|------------------|
| **高度超调（向上）** | current_height > target + 10mm | `height_overshoot` |
| **高度超调（向下）** | current_height < target - 10mm | `height_undershoot` |
| **力超调（向上）** | current_force > target + 150N | `force_overshoot` |
| **力超调（向下）** | current_force < target - 150N | `force_undershoot` |

**Emergency触发流程**：
1. 检测到超限 → 记录错误日志
2. 调用 `_trigger_emergency_reset(reason)` → 执行完整6步reset流程
3. Task状态标记为 `emergency_stop`，completion_reason记录具体原因
4. 系统完全停止，所有继电器清零，释放system_busy锁

## Control Loop线程安全

Control loop (50Hz) 运行在定时器线程中，使用 `control_lock` 保证线程安全：

**每个cycle的检查顺序**：
```python
1. 检查 reset_in_progress
   ├─ True  → 立即返回（不执行任何控制逻辑）
   └─ False → 继续
   
2. 检查互斥冲突
   ├─ control_enabled + force_control_active > 1
   │  └─ Emergency stop，禁用所有控制
   └─ OK → 继续
   
3. 执行控制逻辑
   ├─ Manual down检测
   ├─ Force control（如果 force_control_active = True）
   └─ Height auto control（如果 control_enabled = True）
```

## 状态机

### Task State
```
idle → running → completed
  ↓              ↓
  └─ emergency_stop
```

- `idle`: 无任务运行
- `running`: 任务执行中（height auto 或 force control）
- `completed`: 任务完成（5秒后自动变为idle）
- `emergency_stop`: 安全检测触发停止

### Completion Reasons
- `target_reached`: 高度/力目标达到
- `force_reached`: 力控制目标达到
- `manual_stop`: 用户手动停止/reset
- `height_overshoot`: 高度超调 >±10mm
- `height_undershoot`: 高度超调 <-10mm
- `force_overshoot`: 力超调 >±150N
- `force_undershoot`: 力超调 <-150N

## 安全检测

### Emergency Stop自动触发Reset

当检测到以下安全超限情况时，系统会**自动触发完整reset流程**（而非简单发送stop命令）：

| 触发条件 | 阈值 | Emergency Reason | 检测位置 |
|---------|------|------------------|---------|
| **高度超调（向上）** | current_height > target + 10mm | `height_overshoot` | Height control loop |
| **高度超调（向下）** | current_height < target - 10mm | `height_undershoot` | Height control loop |
| **力超调（向上）** | current_force > target + 150N | `force_overshoot` | Force control loop |
| **力超调（向下）** | current_force < target - 150N | `force_undershoot` | Force control loop |

**Emergency触发流程**：
1. **检测超限** → 记录错误日志（🚨 EMERGENCY）
2. **调用内部reset** → `_trigger_emergency_reset(reason)` 执行完整7步reset流程
   - Step 1-2: 设置flag并等待control loop退出
   - Step 3: 禁用所有控制模式
   - Step 3.5: 取消所有活动Timer
   - Step 4: **清零所有继电器**（relays → OFF）
   - Step 5: **发送STOP脉冲**（触发硬件停止）
   - Step 6: 标记task为`emergency_stop`，设置completion_reason，释放system_busy锁
3. **系统状态** → `task_state = 'emergency_stop'`，所有继电器清零，系统完全停止

**Reset的统一状态**：
- 所有reset操作（无论手动还是自动触发）最终都将task_state设置为`emergency_stop`
- **手动reset**（HTTP API）：reason → `manual_stop`
- **安全超限reset**：reason → 具体原因（`force_overshoot`, `height_overshoot`, `force_undershoot`, `height_undershoot`）
- 通过`completion_reason`可以区分reset的原因

### 高度控制安全（±10mm）
```python
if self.movement_state == 'up':
    if current_height > target_height + 10.0:
        _trigger_emergency_reset('height_overshoot')
elif self.movement_state == 'down':
    if current_height < target_height - 10.0:
        _trigger_emergency_reset('height_undershoot')
```

### 力控制安全（±150N）
```python
if self.force_control_direction == 'up':
    if current_force > target_force + 150.0:
        _trigger_emergency_reset('force_overshoot')
elif self.force_control_direction == 'down':
    if current_force < target_force - 150.0:
        _trigger_emergency_reset('force_undershoot')
```

## 使用示例

### 1. 高度控制
```python
# 通过HTTP API
POST /api/cmd
{
  "command": "goto_height",
  "target": "platform",
  "target_height": 750.0
}

# 内部状态变化
control_enabled = True
force_control_active = False  # 自动禁用
```

### 2. 力控制
```python
# 通过HTTP API
POST /api/cmd
{
  "command": "force_up",
  "target": "platform",
  "target_force": 460.0
}

# 内部状态变化
force_control_active = True
control_enabled = False  # 自动禁用
```

### 3. 紧急复位
```python
# 通过HTTP API
POST /api/cmd
{
  "command": "reset",
  "target": "platform"
}

# 或通过Python
executor.lift_reset_all()
```

## 注意事项

1. ✅ **互斥保证**：Platform的height auto和force control绝对互斥
2. ✅ **线程安全**：Reset命令使用control_lock和reset_in_progress flag保证安全
3. ✅ **时序保证**：Reset等待20ms确保control loop完成当前cycle，然后取消所有Timer
4. ✅ **独立节点**：Pushrod在独立节点中运行，与Platform不冲突
5. ⚠️ **Reset优先级**：Reset是最高优先级，立即停止所有控制
