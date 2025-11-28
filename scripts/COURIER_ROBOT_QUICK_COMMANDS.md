# CourierRobot 快速测试命令

所有命令在 `/home/robot/Desktop/robot_dc_test` 目录下执行

**重要提示**：从现在开始，所有命令会自动打印执行结果，无需手动添加 `print`！

## 1. 状态查询命令

### 查看完整状态（平台+传感器）
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.get_status()"
```

### 只看高度和力
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); s=r.get_status(); print(f\"高度:{s['sensors']['height']}mm, 力:{s['sensors']['combined_force']}N\")"
```

### 查看平台运动状态
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); s=r.get_status(); print(f\"任务状态:{s['data']['platform']['task_state']}, 运动状态:{s['data']['platform']['movement_state']}\")"
```

### 查看推杆状态
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); s=r.get_status(); print(f\"推杆状态:{s['data']['pushrod']['task_state']}, 运动:{s['data']['pushrod']['movement_state']}\")"
```

## 2. 平台手动控制命令

### 平台上升
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_up()"
```

### 平台下降
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_down()"
```

### 平台停止
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_stop()"
```

## 3. 平台高度控制命令

### 移动到指定高度 (800mm)
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_goto_height(800)"
```

### 移动到高度并等待完成 (850mm)
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_goto_height(850); r.wait_for_completion('platform', timeout=30)"
```

### 移动到最低位置 (700mm)
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_goto_height(700)"
```

### 移动到最高位置 (900mm)
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_goto_height(900)"
```

## 4. 平台力控制命令

### 力控向上到 50N
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_force_up(50)"
```

### 力控向下到 30N
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_force_down(30)"
```

### 力控向下到 20N 并等待完成
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_force_down(20); r.wait_for_completion('platform', timeout=30)"
```

## 5. 平台混合控制命令

### 混合控制: 800mm OR 40N (先到先停)
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_hybrid_control(800, 40)"
```

### 混合控制并等待完成
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_hybrid_control(850, 35); r.wait_for_completion('platform', timeout=30)"
```

## 6. 推杆控制命令

### 推杆上升
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.pushrod_up()"
```

### 推杆下降
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.pushrod_down()"
```

### 推杆停止
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.pushrod_stop()"
```

### 推杆绝对定位到 750mm
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.pushrod_goto_height(750, mode='absolute')"
```

### 推杆相对移动 +10mm
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.pushrod_goto_height(10, mode='relative')"
```

### 推杆相对移动 -5mm
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.pushrod_goto_height(-5, mode='relative')"
```

## 7. 紧急复位命令

### 紧急停止并复位所有状态
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.emergency_reset()"
```

## 8. 组合测试命令（简化版）

### 完整流程测试: 移动到目标高度并验证
```bash
python3 -c "
from scripts.courier_robot import CourierRobot
r = CourierRobot()
r.platform_goto_height(820)
r.wait_for_completion('platform', timeout=30)
"
```

### 测试力控精度
```bash
python3 -c "
from scripts.courier_robot import CourierRobot
r = CourierRobot()
r.platform_force_down(25)
r.wait_for_completion('platform', timeout=30)
"
```

### 连续监控传感器数据 (10次，间隔1秒)
```bash
python3 -c "
from scripts.courier_robot import CourierRobot
import time
r = CourierRobot()
for i in range(10):
    s = r.get_status()
    print(f'{i+1}. 高度:{s[\"sensors\"][\"height\"]:.2f}mm, 力:{s[\"sensors\"][\"combined_force\"]:.2f}N')
    time.sleep(1)
"
```

### 测试状态检查机制
```bash
python3 -c "
from scripts.courier_robot import CourierRobot
r = CourierRobot()
r.platform_goto_height(800)  # 第一个命令
r.platform_goto_height(900)  # 第二个命令（会被拒绝）
r.wait_for_completion('platform', timeout=30)
r.platform_goto_height(900)  # 第三个命令（成功）
"
```

## 9. 静默模式（不显示自动输出）

如果你不想看到自动打印的信息，可以设置 `verbose=False`：

```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(verbose=False); r.platform_goto_height(800)"
```

## 10. 常用快捷命令别名

可以在 `~/.bashrc` 中添加以下别名：

```bash
# 添加到 ~/.bashrc
alias robot_status='cd /home/robot/Desktop/robot_dc_test && python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.get_status()"'

alias robot_up='cd /home/robot/Desktop/robot_dc_test && python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_up()"'

alias robot_down='cd /home/robot/Desktop/robot_dc_test && python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_down()"'

alias robot_stop='cd /home/robot/Desktop/robot_dc_test && python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_stop()"'

alias robot_reset='cd /home/robot/Desktop/robot_dc_test && python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.emergency_reset()"'
```

然后执行：
```bash
source ~/.bashrc
```

使用别名（可在任意目录执行）：
```bash
robot_status          # 查看状态
robot_up              # 平台上升
robot_down            # 平台下降
robot_stop            # 平台停止
robot_reset           # 紧急复位
```

## 11. 交互式 Python 使用

如果需要多次操作，推荐进入 Python 交互模式：

```bash
cd /home/robot/Desktop/robot_dc_test
python3
```

```python
from scripts.courier_robot import CourierRobot
r = CourierRobot()

# 所有命令自动显示结果
r.get_status()
r.platform_goto_height(800)
r.wait_for_completion('platform', timeout=30)

# 如果不想看到自动输出
r_quiet = CourierRobot(verbose=False)
result = r_quiet.platform_goto_height(850)
print(result)  # 手动打印你需要的信息
```

## 注意事项

1. **自动输出**: 默认 `verbose=True`，所有操作会自动打印友好的状态信息

2. **静默模式**: 设置 `verbose=False` 可关闭自动输出，适合在脚本中使用

3. **状态检查**: 除了 `stop` 和 `emergency_reset`，所有运动命令都会检查状态，只有在 `idle` 或 `completed` 时才会执行

4. **返回值**: 所有命令都返回包含 `status` 字段的完整状态字典，可以用于编程判断

5. **base_url**: 默认是 `http://192.168.1.3:8090`，如需修改：
   ```bash
   python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(base_url='http://localhost:8090'); r.platform_up()"
   ```

6. **超时设置**: `wait_for_completion()` 默认超时 60 秒，可自定义：
   ```bash
   python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_goto_height(800); r.wait_for_completion('platform', timeout=45)"
   ```
