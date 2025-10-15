# Lift Robot Pushrod Controller

推杆控制器：使用 Modbus 继电器 CH5 / CH6 组合控制推杆升降。

## 通道映射 (Relay -> 含义)
- CH5 = 1, CH6 = 0 : 推杆上升 (up)
- CH5 = 0, CH6 = 1 : 推杆下降 (down)
- 其它组合 (00 / 11) : 推杆停止 (stop)

实现方式：写单线圈 (FC05) 值 0xFF00 打开，0x0000 关闭。

## 启动前准备
确保 `modbus_driver` 已正常运行并提供服务（与平台控制相同的方式）。

## 编译
```bash
cd ~/Documents/robot_dc/colcon_ws
colcon build --packages-select lift_robot_pushrod
source install/setup.bash
```

## 启动节点
```bash
ros2 launch lift_robot_pushrod lift_robot_pushrod_launch.py
```

参数 (可在 launch 文件中修改):
- device_id: Modbus 设备地址 (默认 50，可按硬件实际设置)
- use_ack_patch: 是否使用 ACK 补丁机制 (默认 True)

## 话题
订阅命令:
- `lift_robot_pushrod/command` (std_msgs/String, JSON 格式)
发布状态:
- `lift_robot_pushrod/status` (std_msgs/String, JSON 状态)

## 命令格式
JSON 字段:
- `command`: up | down | stop | timed_up | timed_down | stop_timed
- `duration`: (timed 命令时可选, 秒)
- `seq_id`: 自定义序列标识（可选）

## 示例命令
### 上升
```bash
ros2 topic pub --once /lift_robot_pushrod/command std_msgs/String 'data: "{\"command\": \"up\"}"'
```
### 下降
```bash
ros2 topic pub --once /lift_robot_pushrod/command std_msgs/String 'data: "{\"command\": \"down\"}"'
```
### 停止
```bash
ros2 topic pub --once /lift_robot_pushrod/command std_msgs/String 'data: "{\"command\": \"stop\"}"'
```
### 定时上升 (2秒)
```bash
ros2 topic pub --once /lift_robot_pushrod/command std_msgs/String 'data: "{\"command\": \"timed_up\", \"duration\": 2}"'
```
### 定时下降 (3秒)
```bash
ros2 topic pub --once /lift_robot_pushrod/command std_msgs/String 'data: "{\"command\": \"timed_down\", \"duration\": 3}"'
```
### 取消定时/紧急停止
```bash
ros2 topic pub --once /lift_robot_pushrod/command std_msgs/String 'data: "{\"command\": \"stop_timed\"}"'
```

## 状态查看
```bash
ros2 topic echo /lift_robot_pushrod/status
```

## 退出
Ctrl+C 停止节点，自动清理定时器并复位继电器组合 (CH5=0, CH6=0)。

## 注意
- 00 或 11 都视为停止
- 若硬件需要复位，可在控制逻辑中添加 reset_all_relays 方法（当前未实现）。
