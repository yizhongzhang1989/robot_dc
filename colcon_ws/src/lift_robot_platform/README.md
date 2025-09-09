# Lift Robot Platform Controller

升降机器人平台控制器，使用标准Modbus继电器控制实现闪开效果。

## 启动系统

```bash
# 编译
colcon build --packages-select lift_robot_platform

# 启动
source install/setup.bash
ros2 launch robot_bringup lift_robot_bringup.py
```

## 控制命令

### 停止
```bash
ros2 topic pub --once /lift_robot_platform/command std_msgs/String 'data: "{\"command\": \"stop\"}"'
```

### 上升
```bash
ros2 topic pub --once /lift_robot_platform/command std_msgs/String 'data: "{\"command\": \"up\"}"'
```

### 下降
```bash
ros2 topic pub --once /lift_robot_platform/command std_msgs/String 'data: "{\"command\": \"down\"}"'
```

## 继电器配置

- 0号继电器：停止 (stop)
- 1号继电器：上升 (up) 
- 2号继电器：下降 (down)

每个命令都会执行继电器闪开操作：开启继电器 → 延时100ms → 关闭继电器