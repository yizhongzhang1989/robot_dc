# Task Completion Flow Analysis

## 任务完成流程分析

### ✅ 已改为新流程（通过 pending_task_completion + _on_flash_complete 完成）

#### 1. **goto_height - 精确到达目标带**
- **位置**: Line 1078-1081
- **触发条件**: `abs_error <= POSITION_TOLERANCE` (0.5mm)
- **流程**: 
  ```python
  self._issue_stop(direction=self.movement_state, reason="target_band", 
                   disable_control=True, complete_task_on_stop='target_reached')
  ```
- **完成时机**: STOP relay OFF 验证完成后
- **同步更新**: `movement_state='stop'` + `task_state='completed'`

#### 2. **goto_height - 向上提前停**
- **位置**: Line 1087-1092
- **触发条件**: `current_height >= target - avg_overshoot_up`
- **流程**: 
  ```python
  self._issue_stop(direction='up', reason=f"early_stop_up(th={threshold_height:.2f})", 
                   disable_control=True, complete_task_on_stop='target_reached')
  ```
- **完成时机**: STOP relay OFF 验证完成后
- **同步更新**: `movement_state='stop'` + `task_state='completed'`

#### 3. **goto_height - 向下提前停**
- **位置**: Line 1095-1100
- **触发条件**: `current_height <= target + avg_overshoot_down`
- **流程**: 
  ```python
  self._issue_stop(direction='down', reason=f"early_stop_down(th={threshold_height:.2f})", 
                   disable_control=True, complete_task_on_stop='target_reached')
  ```
- **完成时机**: STOP relay OFF 验证完成后
- **同步更新**: `movement_state='stop'` + `task_state='completed'`

---

### ❌ 仍使用旧流程（立即 controller.stop() + movement_state='stop' + _complete_task()）

#### 4. **manual_down - 触及底部限位**
- **位置**: Line 904-908
- **触发条件**: `current_height < platform_range_min + 1.0`
- **流程**: 
  ```python
  self.controller.stop()
  self.movement_state = 'stop'
  self._complete_task('target_reached')
  ```
- **问题**: 
  - `movement_state='stop'` 在 relay 验证前设置（不准确）
  - `task_state='completed'` 在 relay 验证前设置
  - **不同步**：task 完成时硬件可能还在运动

#### 5. **manual_up - 触及顶部限位**
- **位置**: Line 925-929
- **触发条件**: `current_height > platform_range_max - 1.0`
- **流程**: 
  ```python
  self.controller.stop()
  self.movement_state = 'stop'
  self._complete_task('target_reached')
  ```
- **问题**: 同 manual_down（不同步）

#### 6. **force_up / force_down - 达到目标力**
- **位置**: Line 990-1001
- **触发条件**: 
  - UP: `current_force_combined >= target_force`
  - DOWN: `current_force_combined <= target_force`
- **流程**: 
  ```python
  self.controller.stop()
  self.force_control_active = False
  self.control_enabled = False
  self.movement_state = 'stop'
  self._complete_task('force_reached')
  ```
- **问题**: 同上（不同步）

#### 7. **range_scan - 找到高低端点**
- **位置**: Line 1496-1501
- **触发条件**: 两个端点都已检测到
- **流程**: 
  ```python
  self.controller.stop()
  self.movement_state = 'stop'
  self._complete_task('target_reached')
  ```
- **问题**: 同上（不同步）

#### 8. **timed_up / timed_down - 定时完成**
- **位置**: Line 1616 (_on_auto_stop_complete 回调)
- **触发条件**: controller 定时器到期
- **流程**: 
  ```python
  self._complete_task('target_reached')
  ```
- **问题**: 
  - 定时器回调时，stop relay 可能还未验证完成
  - `movement_state` 可能仍为 'up'/'down'

#### 9. **紧急停止 - 互斥冲突**
- **位置**: Line 877 (control_loop)
- **触发条件**: 多个控制模式同时激活
- **流程**: 
  ```python
  self.movement_state = 'stop'
  self.controller.stop()
  ```
- **问题**: 
  - 紧急停止，不调用 `_complete_task()`
  - `movement_state='stop'` 在 relay 验证前设置

#### 10. **紧急停止 - 安全限位超调**
- **位置**: Line 1656 (_trigger_emergency_reset)
- **触发条件**: 超调超过阈值
- **流程**: 
  ```python
  self.controller.stop()
  self.task_state = 'emergency_stop'
  ```
- **问题**: 同上

---

## 流程时序对比

### 新流程（goto_height）
```
检测到停止条件
  ↓
_issue_stop(complete_task_on_stop='target_reached')
  ↓
pending_task_completion = {'reason': 'target_reached'}
  ↓
controller.stop() 启动异步验证
  ↓
STOP relay ON (发送停止信号)
  ↓
STOP relay OFF (停止信号结束)
  ↓ (~40-60ms)
验证 relay OFF 成功 ✅ ← 硬件真正停止
  ↓
_flash_complete() 触发
  ↓
_on_flash_complete(relay=0) 回调
  ↓ (同步更新)
movement_state = 'stop'
_complete_task('target_reached')
task_state = 'completed'
```

### 旧流程（manual/force/range_scan）
```
检测到停止条件
  ↓
controller.stop() 启动异步验证
  ↓ (立即，不等待)
movement_state = 'stop'  ← 不准确！relay 还在验证
_complete_task('target_reached')
task_state = 'completed'  ← 不准确！硬件可能还在运动
  ↓ (~40-60ms 后)
STOP relay ON
  ↓
STOP relay OFF
  ↓
验证 relay OFF 成功 ✅ ← 此时才真正停止
  ↓
_on_flash_complete(relay=0) 回调
  ↓
movement_state 已经是 'stop'（无更新）
无 pending_task_completion（无操作）
```

---

## 建议修改

### 需要改为新流程的控制模式

1. **manual_down/manual_up - 限位触发**
   - 修改为调用 `_issue_stop(complete_task_on_stop='target_reached')`
   - 移除立即设置 `movement_state='stop'`

2. **force_up/force_down - 力控达标**
   - 修改为调用 `_issue_stop(complete_task_on_stop='force_reached')`
   - 保留禁用控制标志
   - 移除立即设置 `movement_state='stop'`

3. **range_scan - 端点检测**
   - 修改为调用 `_issue_stop(complete_task_on_stop='target_reached')`
   - 移除立即设置 `movement_state='stop'`

4. **timed_up/timed_down** (需特殊处理)
   - 定时器在 controller 内部，回调时机不确定
   - 可能需要修改 controller 的定时器逻辑

### 可以保持旧流程的情况

- **紧急停止 (emergency_stop)**: 紧急情况，不需要精确同步
- **手动 stop 命令**: 已在 command_callback 中单独处理

---

## 关键差异总结（✅ 已完成修改）

| 控制模式 | 当前流程 | 同步性 | 状态 |
|---------|---------|--------|-----|
| **goto_height (3种停止)** | ✅ 新流程 | ✅ 同步 | ✅ 已实现 |
| **manual_up/down 限位** | ✅ 新流程 | ✅ 同步 | ✅ 已修改 |
| **force_up/down** | ✅ 新流程 | ✅ 同步 | ✅ 已修改 |
| **range_scan** | ✅ 新流程 | ✅ 同步 | ✅ 已修改 |
| **手动 stop 命令** | ✅ 新流程 | ✅ 同步 | ✅ 已修改 |
| **timed_up/down** | ❌ 旧流程 | ❌ 不同步 | ⏸️ 暂不修改（controller内部） |
| **emergency_stop** | ❌ 旧流程 | ⚠️ 紧急 | ⏸️ 保持（使用emergency_reset）|

---

## 修改总结（2025-11-20 完成）

### ✅ 已统一为新流程的控制模式（8 个）

所有正常停止流程现已使用 `_issue_stop(complete_task_on_stop=...)` 机制：

1. **goto_height - 精确到达目标带** (Line ~1078)
   - 完成原因: `'target_reached'`
   - 方向: `self.movement_state`

2. **goto_height - 向上提前停** (Line ~1087)
   - 完成原因: `'target_reached'`
   - 方向: `'up'`

3. **goto_height - 向下提前停** (Line ~1095)
   - 完成原因: `'target_reached'`
   - 方向: `'down'`

4. **manual_down - 触及底部限位** (Line ~895) ✅ 新修改
   - 完成原因: `'target_reached'`
   - 方向: `'down'`
   - 说明: 软件限位保护（actual_min + 1mm），不是紧急情况

5. **manual_up - 触及顶部限位** (Line ~920) ✅ 新修改
   - 完成原因: `'target_reached'`
   - 方向: `'up'`
   - 说明: 软件限位保护（actual_max - 1mm），不是紧急情况

6. **force_up/down - 达到目标力** (Line ~987) ✅ 新修改
   - 完成原因: `'force_reached'`
   - 方向: `self.force_control_direction`

7. **range_scan - 找到高低端点** (Line ~1493) ✅ 新修改
   - 完成原因: `'target_reached'`
   - 方向: `self.range_scan_direction`

8. **手动 stop 命令** (Line ~424) ✅ 新修改
   - 完成原因: `'manual_stop'` (仅当有运行中的任务)
   - 方向: `self.movement_state` (fallback to 'up')

### ❌ 保持旧流程的控制模式（2 个）

1. **timed_up/timed_down** - 定时器在 controller 内部
   - 原因: 需要修改 controller 的定时器回调机制
   - 影响: 较小（使用频率低）

2. **emergency_stop** - 紧急停止场景
   - 原因: 使用 `_trigger_emergency_reset()` 独立处理
   - 影响: 无（紧急情况不需要精确同步）

