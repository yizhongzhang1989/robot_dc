# Web与Node交互接口分析及Action-Only适配方案

## 📡 当前Web与lift_robot_node的交互方式

### 1. 数据订阅 (Web读取Node状态)

| Topic | 用途 | 数据内容 |
|-------|------|---------|
| `/lift_robot_platform/status` | 平台状态监控 | `task_state`, `task_type`, `movement_state`, `task_start_time`, `task_end_time`, `completion_reason` |
| `/draw_wire_sensor/data` | 高度传感器数据 | `height`, `register_1` (原始值) |
| `/force_sensor` | 右侧力传感器 | `force` (Float32) |
| `/force_sensor_2` | 左侧力传感器 | `force` (Float32) |
| `/lift_robot_pushrod/status` | 推杆状态 | pushrod相关状态 |

### 2. 命令发布 (Web控制Node)

#### Topic方式 (旧架构主要使用)
**Topic**: `/lift_robot_platform/command`

**命令列表**:
```json
// 手动控制
{"command": "up"}
{"command": "down"}
{"command": "stop"}

// 自动高度控制
{"command": "goto_height", "target_height": 1000.0}

// 力控制
{"command": "force_up", "target_force": 500.0}
{"command": "force_down", "target_force": 300.0}

// 混合控制
{"command": "height_force_hybrid", "target_height": 1200.0, "target_force": 600.0}

// 系统控制
{"command": "reset"}
```

#### Action方式 (已部分实现GotoHeight)
**Action Server**: `/lift_robot_platform/goto_height`

**Web端点**:
- `POST /api/action/goto_height` - 发送目标
- `POST /api/action/cancel_goto_height` - 取消执行
- `GET /api/action/status` - 轮询状态

### 3. Web UI控制面板

根据代码分析，Web界面有以下控制元素需要支持：

1. **手动控制按钮**: UP, DOWN, STOP
2. **高度控制**: 输入框 + GoTo按钮
3. **力控制**: 输入框 + ForceUp/ForceDown按钮
4. **混合控制**: 双输入框 + Hybrid按钮
5. **状态显示**: 实时显示 `task_state`, `movement_state`, 高度, 力值
6. **校准功能**: overshoot校准, 力传感器校准, 高度传感器校准

---

## 🔄 适配方案：Web与lift_robot_node_action对接

### 策略：保持Web代码不变，创建适配层

由于Web已经实现了：
- ✅ Action Client for GotoHeight (已存在)
- ✅ Topic Publisher for command (已存在)
- ✅ Status subscription (已存在)

我们需要：
1. **扩展Web的Action Client**支持所有4种Action
2. **在lift_robot_node_action添加Topic兼容层**（可选，用于平滑迁移）

### 方案A：纯Action方式（推荐）

#### 新增Web Action Clients

```python
# 在LiftRobotWeb.__init__()中添加
from lift_robot_interfaces.action import ForceControl, HybridControl, ManualMove

self.force_control_client = ActionClient(self, ForceControl, '/lift_robot/force_control')
self.hybrid_control_client = ActionClient(self, HybridControl, '/lift_robot/hybrid_control')
self.manual_move_client = ActionClient(self, ManualMove, '/lift_robot/manual_move')
```

#### Web API端点修改

**修改 `/api/cmd` 端点**，将Topic命令转换为Action调用：

```python
@app.post('/api/cmd')
async def send_cmd(payload: dict):
    cmd = payload.get('command')
    
    if cmd in ['up', 'down']:
        # 手动控制 -> ManualMove Action
        goal = ManualMove.Goal()
        goal.direction = cmd
        self.manual_move_client.send_goal_async(goal, feedback_callback=...)
        return {'status': 'action_sent', 'command': cmd}
    
    elif cmd == 'stop':
        # 取消当前Action
        if self.active_goal_handle:
            self.active_goal_handle.cancel_goal_async()
        return {'status': 'cancelled'}
    
    elif cmd == 'goto_height':
        # 高度控制 -> GotoHeight Action (已实现)
        goal = GotoHeight.Goal()
        goal.target_height = payload['target_height']
        self.action_client.send_goal_async(goal, ...)
        return {'status': 'action_sent'}
    
    elif cmd in ['force_up', 'force_down']:
        # 力控制 -> ForceControl Action
        goal = ForceControl.Goal()
        goal.target_force = payload['target_force']
        goal.direction = 'up' if cmd == 'force_up' else 'down'
        self.force_control_client.send_goal_async(goal, ...)
        return {'status': 'action_sent'}
    
    elif cmd == 'height_force_hybrid':
        # 混合控制 -> HybridControl Action
        goal = HybridControl.Goal()
        goal.target_height = payload['target_height']
        goal.target_force = payload['target_force']
        goal.direction = 'up'  # 根据逻辑判断
        self.hybrid_control_client.send_goal_async(goal, ...)
        return {'status': 'action_sent'}
```

#### 状态Topic适配

新节点`lift_robot_node_action`需要发布`/lift_robot_platform/status`以兼容Web监控：

```python
# 在lift_robot_node_action中添加
def _publish_status(self):
    with self.state_lock:
        # 从当前Action状态推断task_state
        task_state = 'idle'
        task_type = None
        
        if self._current_action:
            task_state = 'running'
            task_type = self._current_action  # 'goto_height', 'force_control', etc.
        
        status = {
            'height': self.current_height,
            'movement_state': self.movement_state,
            'force': self.current_force_combined or 0.0,
            'task_state': task_state,
            'task_type': task_type,
            'task_start_time': self._action_start_time,
            'task_end_time': self._action_end_time,
            'completion_reason': self._last_completion_reason
        }
    
    msg = String()
    msg.data = json.dumps(status)
    self.status_publisher.publish(msg)
```

### 方案B：Topic兼容层（过渡方案）

在`lift_robot_node_action`中添加Topic订阅，将Topic命令转换为Action调用：

```python
def command_callback(self, msg):
    """Topic兼容层：将旧的Topic命令转换为Action调用"""
    try:
        data = json.loads(msg.data)
        cmd = data.get('command')
        
        if cmd == 'goto_height':
            # 内部触发GotoHeight Action
            self._trigger_goto_height_action(data['target_height'])
        elif cmd == 'force_up':
            self._trigger_force_control_action(data['target_force'], 'up')
        # ... 其他命令映射
    except Exception as e:
        self.get_logger().error(f"Command callback error: {e}")
```

---

## ✅ 推荐实施步骤

### 阶段1：扩展Web的Action支持（核心修改）

1. **修改 `/api/cmd`** 端点，路由到对应Action
2. **添加3个新Action Client**: ForceControl, HybridControl, ManualMove
3. **统一Action状态管理**，支持多个并发Action
4. **测试Web控制面板**所有按钮功能

### 阶段2：增强lift_robot_node_action状态发布

1. **完善status topic**，包含所有Web需要的字段
2. **跟踪当前Action状态**，映射为task_state
3. **保持向后兼容**，确保旧Web也能读取

### 阶段3：可选Topic兼容层

如果需要支持旧版Web或其他系统：
1. 添加`/lift_robot_platform/command`订阅
2. Topic命令内部转为Action调用
3. 保持接口一致性

---

## 📋 需要修改的文件清单

### Web端 (lift_robot_web/server.py)

```python
# 需要修改的部分
1. __init__(): 添加3个新Action Client
2. send_cmd(): 改造命令路由逻辑
3. 添加4个新callback: force_control, hybrid, manual_move回调
4. 统一Action状态管理
```

### Node端 (lift_robot_node_action.py)

```python
# 需要添加的部分
1. _current_action: 跟踪当前执行的Action类型
2. _action_start_time, _action_end_time: 任务时间戳
3. _last_completion_reason: 完成原因
4. _publish_status(): 增强状态字段
5. (可选) command_callback(): Topic兼容层
```

---

## 🎯 最小改动方案（快速上线）

如果希望最小化改动：

1. **只修改Web的`/api/cmd`端点** - 将命令路由到Action
2. **lift_robot_node_action添加少量状态变量** - 用于status发布
3. **保持所有其他Web代码不变**

预计修改行数：
- Web端：~100行
- Node端：~50行

---

## 🔍 关键差异对比

| 特性 | Topic方式 | Action方式 |
|------|----------|-----------|
| 命令发送 | 发后即忘 | 带Goal确认 |
| 状态监控 | 轮询status topic | Feedback实时推送 |
| 取消操作 | 发送stop命令 | cancel_goal() |
| 进度反馈 | 无标准化 | progress字段 (0-100%) |
| 完成确认 | task_state轮询 | Result回调 |
| 并发控制 | 需自行管理 | 框架内置 |

---

## 🚀 实施建议

**推荐采用方案A（纯Action）+ 阶段1+2**：

- ✅ 架构清晰，Action-only
- ✅ 充分利用Action框架特性
- ✅ Web改动集中在一个文件
- ✅ 保持UI/UX完全不变

需要我开始创建具体的修改代码吗？
