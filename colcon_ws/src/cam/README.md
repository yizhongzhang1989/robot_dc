# cam

摄像头抓拍节点，调用 snapshot 服务，具备防卡死和自动重启摄像头功能。

## 依赖
- Python requests
- 需先运行 scripts/rtsp_cam_snapshot_server.py 提供 HTTP 拍照服务

## 构建
在 ROS2 工作空间根目录下执行：

```bash
colcon build --packages-select cam
source install/setup.bash
```

## 运行
### 直接运行节点
```bash
ros2 run cam cam_node
```

### 使用 launch 文件
```bash
ros2 launch cam cam_launch.py
```

## 节点说明
- 节点会定时（默认30秒）请求 cam100 和 cam101 两个摄像头的抓拍服务。
- 若抓拍失败会自动重试，连续失败后会调用 restart_camera（需根据实际硬件实现重启逻辑）。

## 注意事项
- 请确保 `scripts/rtsp_cam_snapshot_server.py` 已在 8012 端口运行。
- 如需修改抓拍间隔或摄像头地址，请编辑 `cam_node.py`。 