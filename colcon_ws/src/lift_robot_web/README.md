# lift_robot_web

最小版实时查看 `/cable_sensor/data` 话题的 Web 可视化。后端使用 FastAPI + WebSocket，通过 ROS2 订阅将最新消息立即广播；前端优先 WebSocket，失败时自动退化为每秒轮询 `/api/latest`。

## 功能
- WebSocket 实时推送传感器 JSON 数据
- 自动回退轮询
- 显示寄存器值、seq、设备号、频率估计、延迟与 STALE 状态
- 原始 JSON 展示，便于调试

## 文件结构
```
lift_robot_web/
  launch/lift_robot_web.launch.py   # ROS2 启动文件
  lift_robot_web/server.py          # FastAPI + rclpy 集成节点
  web/index.html                    # 前端页面
  package.xml / setup.py            # ROS2 Python 包元数据
```

## 构建
在工作区 `colcon_ws` 下：
```bash
cd /home/robot/Documents/robot_dc/colcon_ws
colcon build --packages-select lift_robot_web
source install/setup.bash
```

## 启动
```bash
ros2 launch lift_robot_web lift_robot_web.launch.py port:=8090 sensor_topic:=/cable_sensor/data
```
然后浏览器访问：`http://<主机IP>:8090/` (根路径会自动返回 index.html)。

> 如果在本机：`http://localhost:8090/`

## 参数
- `port` (int, 默认 8090): HTTP / WebSocket 监听端口
- `sensor_topic` (string, 默认 `/cable_sensor/data`): 要订阅的传感器话题

## API & Endpoint
- `GET /api/latest` : 返回最近一次消息（JSON 或 404）
- `WS  /ws` : WebSocket 实时推送（文本 JSON）。服务端每 30s 发送一次 `ping` 字符串保持连接。
- 静态资源：`/` -> `web/index.html`

## 前端 STALE 判定
- 根据最后接收时间 age（ms）与动态阈值：`max(2500, 2.5 * read_interval)`
- 消息含 `read_interval` 字段 (秒) 时会更新频率估计；若当前发布节点没有该字段，则前端使用默认 1000ms 周期推导阈值。

## 可能的改进（可选）
- 增加平台 up/down/stop 命令接口：新增 `POST /api/cmd {"command":"up"}` 并发布到 `/lift_robot_platform/command`
- 在传感器消息中加入 `sent_ts`, `read_interval` 字段（需修改传感器节点）
- 支持跨域：添加 `CORSMiddleware`
- 增加基本鉴权（Token / Basic Auth）

## 故障排查
| 现象 | 排查步骤 |
|------|----------|
| 页面一直 Disconnected | 确认端口监听：`ss -ltnp | grep 8090`; 浏览器/网络防火墙 |
| WebSocket 频繁重连 | 查看服务器日志是否异常退出；确认带宽/代理；尝试关闭 VPN |
| 数据长时间 Stale | 确认发布节点是否仍在发：`ros2 topic echo /cable_sensor/data`；检查频率是否下降 |
| 404 /api/latest | 尚未接收到任何消息；等待或检查传感器节点发布 |

## License
内部使用（未声明公共许可证）。
