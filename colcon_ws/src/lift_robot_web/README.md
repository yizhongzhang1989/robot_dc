# lift_robot_web

Minimal real-time web visualization of the `/cable_sensor/data` (now `/draw_wire_sensor/data`) topic. Backend: FastAPI + WebSocket broadcasting newest message on ROS2 subscription; frontend prefers WebSocket and falls back to 1s polling `/api/latest` if the socket fails.

## Features
- WebSocket real‑time push of sensor JSON data
- Automatic fallback polling
- Displays register values, seq, device id, frequency estimate, latency & STALE state
- Raw JSON panel for debugging

## Structure
```
lift_robot_web/
  launch/lift_robot_web.launch.py   # ROS2 launch file
  lift_robot_web/server.py          # FastAPI + rclpy integrated node
  web/index.html                    # Frontend page
  package.xml / setup.py            # Package metadata
```

## Build
In workspace `colcon_ws`:
```bash
cd /home/robot/Documents/robot_dc/colcon_ws
colcon build --packages-select lift_robot_web
source install/setup.bash
```

## Launch
```bash
ros2 launch lift_robot_web lift_robot_web.launch.py port:=8090 sensor_topic:=/cable_sensor/data
```
Then open: `http://<HOST_IP>:8090/` (root serves index.html)

> Local: `http://localhost:8090/`

## Parameters
- `port` (int, default 8090): HTTP / WebSocket listen port
- `sensor_topic` (string, default `/draw_wire_sensor/data`): subscribed sensor topic

## API & Endpoints
- `GET /api/latest` : latest message (JSON or 404)
- `WS  /ws` : WebSocket real‑time push (text JSON). Server sends a `ping` every 30s.
- Static: `/` -> `web/index.html`

## Frontend STALE Logic
- Uses last receive age (ms) vs dynamic threshold: `max(2500, 2.5 * read_interval)`
- If message includes `read_interval` (s) it refines frequency estimate; else assumes 1000ms period.

## Possible Enhancements
- Platform up/down/stop command API (already implemented: `POST /api/cmd {"command":"up"}` -> `/lift_robot_platform/command`)
- Add `sent_ts` field to sensor message
- CORS support (FastAPI `CORSMiddleware`)
- Basic auth / token auth

## Troubleshooting
| Symptom | Action |
|---------|--------|
| Page stays Disconnected | Check port: `ss -ltnp | grep 8090`; browser/network firewall |
| Frequent WS reconnects | Inspect server logs; bandwidth/proxy/VPN issues |
| Data long STALE | `ros2 topic echo /draw_wire_sensor/data`; check frequency drop |
| 404 /api/latest | No message yet; wait or verify publisher |

## License
Internal use (no public license declared).
