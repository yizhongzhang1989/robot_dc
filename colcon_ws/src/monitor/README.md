# Robot Monitor Node

## 概述

Robot Monitor Node 是一个 ROS2 节点，用于监听机器人的 UDP 数据流并自动记录到 rosbag 文件。该节点实现了与 `dk_test.py` 脚本相同的功能，但增加了 ROS2 集成和自动数据记录功能。

## 功能特性

- ✅ **UDP 数据监听**: 同时监听端口 5566（机器人数据）和 5577（日志数据）
- ✅ **端口共享**: 使用 SO_REUSEPORT 与 robot_arm_web_server 共享端口 5566
- ✅ **自动时间戳**: 为所有接收的数据添加纳秒级时间戳
- ✅ **ROS2 话题发布**: 
  - `/robot_data` - 机器人状态数据（TCP位置、关节角度、力传感器等）
  - `/log_data` - 系统日志和状态消息（可选，用于调试和状态监控）
- ✅ **自动 rosbag 记录**: 自动启动 rosbag 录制并保存到指定目录
- ✅ **可配置存储**: 支持自定义数据存储位置
- ✅ **会话信息**: 自动生成会话元数据文件
- ✅ **数据压缩**: 使用 zstd 压缩节省存储空间

## 系统要求

- ROS2 Humble
- Python 3.8+
- 支持 SO_REUSEPORT 的 Linux 系统

## 快速开始

### 1. 构建和安装

```bash
# 进入工作空间并构建
cd /path/to/robot_dc3/colcon_ws
colcon build --packages-select monitor
source install/setup.bash
```

### 2. 启动监控

```bash
# 使用默认存储位置 (~/robot_data)
ros2 launch monitor monitor.launch.py

# 指定自定义存储位置
ros2 launch monitor monitor.launch.py data_dir:=/your/custom/path

# 或设置环境变量
export ROBOT_DATA_DIR=/your/custom/path
ros2 launch monitor monitor.launch.py
```

### 3. 数据管理

```bash
# 查看所有会话
python3 scripts/robot_monitor_manager.py --list --detailed

# 分析特定会话
python3 scripts/robot_monitor_manager.py --analyze ~/robot_data/2025-08-25/robot_monitor_HHMMSS

# 查看数据内容
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-08-25/robot_monitor_HHMMSS --limit 5 --full

# 查看原始二进制数据格式
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-08-25/robot_monitor_HHMMSS --limit 2 --raw
```

## 运行时信息

### 节点启动日志

Monitor节点启动后会在终端实时显示以下信息：

```
[INFO] [robot_monitor_node]: Robot Monitor Node started
[INFO] [robot_monitor_node]: Data will be saved to: /home/user/robot_data
[INFO] [robot_monitor_node]: Rosbag will be saved to: /home/user/robot_data/2025-08-25/robot_monitor_134753
[INFO] [robot_monitor_node]: Session info saved to: /home/user/robot_data/2025-08-25/robot_monitor_134753_info.json
[INFO] [robot_monitor_node]: Started rosbag recording with PID: 12345
[INFO] [robot_monitor_node]: Listening on ports 5566 (data) and 5577 (log)
[INFO] [robot_monitor_node]: Using SO_REUSEPORT to share port 5566 with robot_arm_web_server
[INFO] [robot_monitor_node]: Data receiver bound to 0.0.0.0:5566
[INFO] [robot_monitor_node]: Log receiver bound to 0.0.0.0:5577
[INFO] [robot_monitor_node]: [DATA] From 192.168.1.100:5566 - RobotTcpPos: [-98.85, 658.5, 500.85, -3.81, 0.13, -170.38]
[INFO] [robot_monitor_node]: [LOG] From 192.168.1.100:5577: System status OK
```

> ✅ **实时监控**: 节点运行时会自动在终端显示接收到的数据，无需额外命令查看实时数据流。

### 数据内容示例

**Robot Data** (`/robot_data` topic) - 主要数据流:
- **RobotTcpPos**: TCP位置 [x, y, z, rx, ry, rz] (mm和度)
- **RobotAxis**: 关节角度 [J1-J6] (度)
- **RobotTrack**: 导轨位置
- **FTSensorData**: 力/扭矩传感器 [Fx, Fy, Fz, Mx, My, Mz]
- **FTTarget**: 力/扭矩目标值

**Log Data** (`/log_data` topic) - 辅助数据流（可选）:
- 系统日志消息和事件
- 错误通知和状态更新
- 调试信息

> 📝 **注意**: 实际使用中主要关注 `/robot_data` 话题，`/log_data` 话题用于系统日志，可能没有数据或数据较少。

## 数据存储

### 存储结构

```
数据根目录/
├── 2025-08-25/
│   ├── robot_monitor_134753/          # rosbag 数据目录
│   │   ├── robot_monitor_134753_0.db3.zstd # 压缩的SQLite数据库文件
│   │   └── metadata.yaml              # rosbag 元数据
│   └── robot_monitor_134753_info.json # 会话信息文件
├── 2025-08-26/
└── ...
```

### 存储位置配置

**方式 1: 环境变量（推荐）**
```bash
export ROBOT_DATA_DIR=/your/custom/path
```

**方式 2: 启动参数**
```bash
ros2 launch monitor monitor.launch.py data_dir:=/your/custom/path
```

**方式 3: 查看当前配置**
```bash
python3 scripts/robot_monitor_manager.py --config
```

### 会话信息文件示例

```json
{
  "session_start": "2025-08-25T13:47:53.123456",
  "bag_name": "robot_monitor_134753",
  "data_sources": {
    "robot_data_port": 5566,
    "log_data_port": 5577
  },
  "topics": ["/robot_data", "/log_data"]
}
```

## 管理工具

### robot_monitor_manager.py - 统一数据管理

**主要功能**: 配置管理、会话列表、数据分析、内容查看

```bash
# 基本操作
python3 scripts/robot_monitor_manager.py --config           # 查看配置
python3 scripts/robot_monitor_manager.py --list --detailed  # 列出会话
python3 scripts/robot_monitor_manager.py --test             # 测试连接

# 数据分析
python3 scripts/robot_monitor_manager.py --analyze ~/robot_data/2025-08-25/robot_monitor_HHMMSS

# 数据查看
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-08-25/robot_monitor_HHMMSS --limit 5 --full
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-08-25/robot_monitor_HHMMSS --topic /log_data  # 查看日志（可能无数据）

# 查看原始数据格式
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-08-25/robot_monitor_HHMMSS --limit 2 --raw  # 十六进制原始数据

# 查看数据库结构和时间戳存储详情
python3 scripts/robot_monitor_manager.py --db-info ~/robot_data/2025-08-25/robot_monitor_HHMMSS

# 数据清理
python3 scripts/robot_monitor_manager.py --cleanup 30          # 预览清理
python3 scripts/robot_monitor_manager.py --cleanup-confirm 30  # 确认清理
```

### test_udp_send.py - 测试数据发送工具

```bash
python3 scripts/test_udp_send.py -n 20 -i 0.5  # 发送20条消息，间隔0.5秒
```

> 💡 **注意**: 此工具用于开发和测试环境，实际部署时机器人会直接发送UDP数据。

## 数据查看

### 实时监控

当monitor节点运行时，所有接收的数据会直接在节点终端实时显示：

```
[INFO] [robot_monitor_node]: [DATA] 15:49:05.844 From 192.168.1.100:5566 - RobotTcpPos: [-41.04, 238.16, 652.56, 87.34, 6.03, 179.41], RobotAxis: [1.15, -0.45, 1.31, 0.81, -1.62, -1.95], FTSensorData: [0.84, 1.02, 29.48, 0.26, -0.56, 0.36]
[INFO] [robot_monitor_node]: [LOG] 15:49:06.123 From 192.168.1.100:5577: System status OK
```

### 时间戳说明

### 时间戳详细说明

**rosbag时间戳存储位置**：
- **表**: `messages`
- **字段**: `timestamp` (INTEGER类型，64位)
- **精度**: 纳秒级 (1756108145806574972 = 2025-08-25 15:49:05.806574972)
- **来源**: ROS2节点接收UDP数据包的系统时间

**Database Field Description**:

| Field Name  | Type      | Purpose                                     | Your Actual Data Example     |
|-------------|-----------|---------------------------------------------|------------------------------|
| `id`        | INTEGER   | Unique message identifier                   | 1, 2, 3...                   |
| `topic_id`  | INTEGER   | Topic ID (1=/robot_data, 2=/log_data)      | 1                            |
| `timestamp` | INTEGER   | **Timestamp (nanoseconds)**                | 1756108145806574972          |
| `data`      | BLOB      | **Robot data (binary)**                    | 232 bytes of robot data      |

**Why Use Reception Timestamp**:
1. Robot's raw data contains no timestamp, only position, angle, force sensor data
2. Reception timestamp more accurately reflects when data was recorded
3. Avoids network delay and clock synchronization issues

**View Complete Database Information**:
```bash
python3 scripts/robot_monitor_manager.py --db-info ~/robot_data/2025-08-25/robot_monitor_HHMMSS
```
Shows: database structure, timestamp storage details, message statistics, recording duration, etc.

**原始数据格式**：
```json
{
  "RobotTcpPos": [-41.04, 238.16, 652.56, 87.34, 6.03, 179.41],
  "RobotAxis": [1.15, -0.45, 1.31, 0.81, -1.62, -1.95],
  "RobotTrack": 0,
  "FTSensorData": [0.85, 1.01, 29.2, 0.26, -0.58, 0.36],
  "FTTarget": [0, 0, 0, 0, 0, 0]
}
```

**存储格式**：
```json
{
  "raw_message": "{原始机器人JSON数据}"
}
```
时间戳存储在rosbag的message header中。

### 查看历史数据

```bash
# 查看数据内容，支持压缩文件
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-08-25/robot_monitor_HHMMSS --limit 5 --full

# 查看原始二进制数据格式 (十六进制和ASCII)
python3 scripts/robot_monitor_manager.py --view ~/robot_data/2025-08-25/robot_monitor_HHMMSS --limit 2 --raw
```

> 💡 **注意**: Monitor节点运行时会在终端实时打印接收的数据：
> - `[DATA] timestamp From IP - RobotTcpPos: [...], RobotAxis: [...], FTSensorData: [...]`
> - `[LOG] timestamp From IP: message`

