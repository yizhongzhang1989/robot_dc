# Lift Robot Cable Sensor

缆线传感器ROS2节点，用于读取升降机器人的缆线传感器数据。

## 功能

- 通过Modbus RTU协议读取传感器数据
- 周期性自动读取并发布传感器数据
- 支持手动命令触发读取

## 硬件配置

- **设备ID**: 51 (0x33 hex)
- **波特率**: 115200
- **Modbus命令**: `33 03 00 00 00 02 +CRC`
- **功能码**: 03 (读取保持寄存器)
- **起始地址**: 0x0000
- **寄存器数量**: 2

## 启动

```bash
# 编译
colcon build --packages-select lift_robot_cable_sensor

# 启动
source install/setup.bash
ros2 launch lift_robot_cable_sensor cable_sensor_launch.py
```

## 使用命令

### 手动读取传感器数据
```bash
ros2 topic pub --once /cable_sensor/command std_msgs/String 'data: "{\"command\": \"read\"}"'
```

### 获取最新数据
```bash
ros2 topic pub --once /cable_sensor/command std_msgs/String 'data: "{\"command\": \"get_data\"}"'
```

### 查看传感器数据
```bash
ros2 topic echo /cable_sensor/data
```

## 数据格式

传感器数据以JSON格式发布到 `/cable_sensor/data` 话题：

```json
{
  "timestamp": 1693123456.789,
  "register_0": 1234,
  "register_1": 5678,
  "device_id": 51,
  "seq_id": 123
}
```

## 参数配置

- `device_id`: Modbus设备ID (默认: 51)
- `read_interval`: 自动读取间隔秒数 (默认: 1.0)
- `use_ack_patch`: 是否使用ACK确认机制 (默认: True)
