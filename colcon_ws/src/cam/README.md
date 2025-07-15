# cam

Camera snapshot node with retry and auto-restart capability.

## Dependencies
- Python requests
- (Legacy) If using HTTP snapshot service: scripts/rtsp_cam_snapshot_server.py must be running on port 8012

## Build
In the ROS2 workspace root directory:

```bash
colcon build --packages-select cam
source install/setup.bash
```

## Run
### Run node directly
```bash
ros2 run cam cam_node
```

### Use launch file
```bash
ros2 launch cam cam_launch.py
```

## Node Description
- The node periodically (default 30s) requests snapshots from cam100 and cam101 RTSP cameras.
- If snapshot fails, it will retry automatically. After several failures, it will call restart_camera (you need to implement actual hardware restart logic if needed).

## Notes
- If using HTTP snapshot service, ensure `scripts/rtsp_cam_snapshot_server.py` is running on port 8012.
- To change snapshot interval or camera addresses, edit `cam_node.py`. 