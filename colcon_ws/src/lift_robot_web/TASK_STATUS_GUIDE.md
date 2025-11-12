# Lift Robot Unified Task Status Display

## 概述

Web 界面显示**统一的任务状态卡片**，合并 Platform 和 Pushrod 的状态，提供整体机器的任务执行状态。

## 统一任务状态卡片

### 显示字段

1. **State (状态)** - 彩色徽章
   - **RUNNING** (蓝色 + 脉冲动画) - 任务正在执行中
   - **COMPLETED** (绿色) - 任务已完成
   - **EMERGENCY_STOP** (红色) - 紧急停止（软件限位触发）

2. **Task Type (任务类型)**
   - 显示当前或最后执行的任务类型
   - 例如：`goto_height`, `force_up`, `force_down`, `manual_up` 等

3. **Duration (持续时间)**
   - 运行中：显示已运行时间（实时更新）
   - 已完成：显示总执行时间

4. **Completion (完成原因)**
   - `target_reached` - 目标高度已到达
   - `force_reached` - 目标力已达到
   - `limit_exceeded` - 超出软件限位
   - `manual_stop` - 手动停止

## 状态合并逻辑

### Running 优先
- Platform 或 Pushrod **任一** 为 `running` → 整体显示 `RUNNING`
- 显示正在运行的那个设备的任务信息

### Emergency Stop 次优先
- 任一设备触发 `emergency_stop` → 整体显示 `EMERGENCY_STOP`

### Completed 默认
- 其他所有情况（包括 idle）→ 显示 `COMPLETED`
- 显示最近完成的任务信息（按 task_end_time 排序）
- **不再显示 IDLE 状态**（除系统刚启动外）

### 信息来源
- **运行中任务**：使用当前运行设备的信息
- **已完成任务**：使用最近完成的设备信息（Platform 和 Pushrod 按结束时间比较）

## 界面布局

```
┌─────────────────────────────────────┐
│ Task Status                         │
├─────────────────────────────────────┤
│ State:      [RUNNING] (蓝色脉冲)    │
│ Task Type:  goto_height             │
│ Duration:   3.5s                    │
│ Completion: -                       │
└─────────────────────────────────────┘
```

### 颜色方案
- **蓝色徽章 + 脉冲动画**：运行中
- **绿色徽章**：已完成
- **红色徽章**：紧急停止

## 使用示例

### 场景 1：Platform 执行 goto_height
```javascript
// Platform: running, Pushrod: idle
// 显示: RUNNING (蓝色)
{
  state: "RUNNING",
  task_type: "goto_height",
  duration: "2.3s",
  completion: "-"
}
```

### 场景 2：Pushrod 执行完成，Platform 空闲
```javascript
// Platform: idle, Pushrod: completed
// 显示: COMPLETED (绿色)
{
  state: "COMPLETED",
  task_type: "goto_height",
  duration: "5.24s",
  completion: "target_reached"
}
```

### 场景 3：Platform 和 Pushrod 都在运行
```javascript
// Platform: running (force_up), Pushrod: running (goto_height)
// 显示: RUNNING (蓝色) - 显示最先开始的任务
{
  state: "RUNNING",
  task_type: "force_up",  // 或 "goto_height"，取决于谁先开始
  duration: "8.7s",
  completion: "-"
}
```

### 场景 4：Platform 触发急停
```javascript
// Platform: emergency_stop, Pushrod: idle
// 显示: EMERGENCY_STOP (红色)
{
  state: "EMERGENCY_STOP",
  task_type: "goto_height",
  duration: "12.45s",
  completion: "limit_exceeded"
}
```

## HTTP API 查询

```python
import requests

# 查询状态
response = requests.get('http://192.168.1.3:8090/api/status')
status = response.json()

# 获取 Platform 和 Pushrod 状态
platform = status['platform']
pushrod = status['pushrod']

# 前端自动合并显示，后端仍返回分开的状态
```

## 技术实现

### JavaScript 函数
```javascript
// 合并 Platform 和 Pushrod 状态
function updateUnifiedTaskStatus(platformStatus, pushrodStatus) {
  // 1. 检查 running 状态（优先级最高）
  // 2. 检查 emergency_stop
  // 3. 默认显示 completed
  // 4. 更新 UI 元素和颜色
}
```

### 后端支持
- `/api/status` 端点返回完整的 Platform 和 Pushrod 状态
- WebSocket 实时推送更新
- 无需修改后端，合并逻辑在前端实现

## 注意事项

1. **简化显示**：不再区分 Platform 和 Pushrod，统一显示整体状态
2. **无 IDLE 状态**：除系统刚启动外，其他时候显示为 COMPLETED
3. **实时更新**：运行中任务的持续时间实时更新（每次 WebSocket 消息）
4. **优先级明确**：running > emergency_stop > completed
5. **信息完整**：显示最相关的任务信息（运行中或最近完成）

## 文件修改

- ✅ `index.html` - 单一任务状态卡片 + 合并逻辑
- ✅ `server.py` - 无需修改（已支持）

