# Robot Web Package

## 概述
此包提供 Web 服务器接口，用于通过 HTTP API 与 ROS2 机器人系统交互，包括相机快照、电机控制等功能。

## 功能
- FastAPI Web 服务器
- 相机快照 HTTP 接口
- 电机状态监控和控制
- ROS2 服务桥接

## 启动命令

### 单独启动 robot_web
```bash
cd /home/jetson/Desktop/robot_dc/colcon_ws
source install/setup.bash
ros2 launch robot_web web_server_launch.py
```

### 同时启动相机和 Web 服务（推荐）
```bash
cd /home/jetson/Desktop/robot_dc/colcon_ws
source install/setup.bash
ros2 launch cam_node camera_web_launch.py
```

### 直接启动 Web 服务器
```bash
cd /home/jetson/Desktop/robot_dc/colcon_ws
source install/setup.bash
uvicorn robot_web.web_server:app --host 0.0.0.0 --port 8000
```

## API 接口

### 相机快照
- **URL**: `POST /snapshot`
- **功能**: 获取相机快照
- **返回**: JSON 格式，包含两个摄像头的 base64 编码图像

#### 测试命令
```bash
curl -X POST http://localhost:8000/snapshot
```

### 电机控制
- **URL**: `POST /api/{motor_id}/cmd`
- **功能**: 发送电机命令
- **参数**: motor_id (电机ID), command (命令), value (值)

### 电机状态
- **URL**: `GET /api/{motor_id}/status`
- **功能**: 获取电机状态

### 所有状态
- **URL**: `GET /api/all_status`
- **功能**: 获取所有电机状态

## Web 界面
- 访问 `http://localhost:8000` 查看 Web 界面
- 支持电机控制和状态监控
- 相机快照功能

## 配置
- 服务器地址: 0.0.0.0:8000
- 支持的电机: motor1, motor2, motor17, motor18, platform

## 依赖
- fastapi
- uvicorn
- rclpy
- std_srvs

## 注意事项
1. 确保 ROS2 环境正确配置
2. 确保相机节点正在运行（如果使用快照功能）
3. 端口 8000 需要可用
4. 启动前需要 source install/setup.bash
