# Cam Node Package

## 概述
此包提供 ROS2 相机快照服务，支持从两个 RTSP 摄像头流（高清和低清）获取图像快照。

## 功能
- 连接两个 RTSP 摄像头流 (192.168.1.100 和 192.168.1.101)
- 提供 ROS2 快照服务 (`/snapshot`)
- 返回 base64 编码的 JPEG 图像
- 自动重连和错误处理

## 启动命令

### 单独启动 cam_node
```bash
cd /home/jetson/Desktop/robot_dc/colcon_ws
source install/setup.bash
ros2 launch cam_node cam_launch.py
```

### 同时启动相机和 Web 服务（推荐）
```bash
cd /home/jetson/Desktop/robot_dc/colcon_ws
source install/setup.bash
ros2 launch cam_node camera_web_launch.py
```

## 服务接口

### /snapshot
- **类型**: std_srvs/srv/Trigger
- **功能**: 获取两个摄像头的快照
- **返回**: JSON 格式的响应，包含两个摄像头的 base64 编码图像

#### 测试命令
```bash
ros2 service call /snapshot std_srvs/srv/Trigger
```

## 配置
- 高清摄像头: rtsp://admin:123456@192.168.1.100/stream0
- 低清摄像头: rtsp://admin:123456@192.168.1.101/stream0

## 依赖
- rclpy
- opencv-python
- numpy
- ffmpeg/ffprobe

## 注意事项
1. 确保两个摄像头网络连接正常
2. 启动前需要 source install/setup.bash
3. 需要安装 ffmpeg 和 ffprobe 工具
