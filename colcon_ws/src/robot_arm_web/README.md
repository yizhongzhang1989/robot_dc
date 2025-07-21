# robot_arm_web

`robot_arm_web` is a ROS 2 package that provides a web-based interface for controlling the DUCO robot arm. It offers a simple, intuitive web dashboard with four essential control buttons for basic robot arm operations.

---

## 📦 Package Structure

```
robot_arm_web/
├── robot_arm_web/
│   ├── web_server_node.py          # Main ROS 2 node with FastAPI web server
│   └── __init__.py
├── web/
│   ├── index.html                  # Main web interface
│   ├── js/
│   │   └── robot_arm_control.js    # JavaScript for web interface
│   └── css/
│       └── style.css               # Additional styling
├── launch/
│   └── robot_arm_web_launch.py     # Launch file
└── test/
    └── test_robot_arm_web.py       # Unit tests
```

---

## ⚙️ Features

* 🖥️ **Web Interface**: Clean, responsive web dashboard
* 🎮 **4 Control Buttons**: Power On, Power Off, Enable, Disable
* 📡 **ROS 2 Integration**: Publishes to robot arm command topic
* 📊 **Real-time Status**: Connection status and command feedback
* 📝 **Command Log**: Track all sent commands with timestamps
* ⌨️ **Keyboard Shortcuts**: Ctrl+1/2/3/4 for quick commands
* 🔄 **Auto-reconnect**: Periodic connection health checks

---

## 🛠️ Dependencies

* **ROS 2** (Humble tested)
* **Python 3.8+**
* **FastAPI** (for web server)
* **Uvicorn** (ASGI server)
* **duco_robot_arm** (for robot arm control)

---

## 🚀 Usage

### 1. Build the Package

```bash
cd ~/colcon_ws
colcon build --packages-select robot_arm_web
source install/setup.bash
```

### 2. Start the Robot Arm Node

First, start the robot arm node:

```bash
ros2 run duco_robot_arm duco_robot_arm_node \
  --ros-args \
  -p ip:=192.168.1.10 \
  -p port:=7003 \
  -p device_id:=1
```

### 3. Launch the Web Interface

Using the launch file:

```bash
ros2 launch robot_arm_web robot_arm_web_launch.py
```

Or run directly:

```bash
ros2 run robot_arm_web robot_arm_web_server \
  --ros-args \
  -p device_id:=1 \
  -p port:=8080
```

### 4. Access the Web Interface

Open your browser and navigate to:
```
http://localhost:8080
```

---

## 🎮 Web Interface

### Control Buttons

| Button      | Command     | Description                | Shortcut |
|-------------|-------------|----------------------------|----------|
| ⚡ Power On | `power_on`  | Power on the robot arm     | Ctrl+1   |
| 🔴 Power Off| `power_off` | Power off the robot arm    | Ctrl+2   |
| ✅ Enable   | `enable`    | Enable robot arm motors    | Ctrl+3   |
| ⏸️ Disable  | `disable`   | Disable robot arm motors   | Ctrl+4   |

### Dashboard Features

* **Status Panel**: Shows connection status and robot arm info
* **Control Panel**: Four main control buttons with visual feedback
* **Command Log**: Real-time log of all sent commands
* **Keyboard Shortcuts**: Quick access via Ctrl+number keys

---

## 🔧 Configuration

### ROS Parameters

| Parameter   | Default | Description                    |
|-------------|---------|--------------------------------|
| `device_id` | `1`     | Robot arm device ID            |
| `port`      | `8080`  | Web server port                |

### Topic Configuration

The web interface publishes to: `/arm{device_id}/cmd`

Example: For device_id=1, publishes to `/arm1/cmd`

---

## 🌐 API Endpoints

### REST API

| Method | Endpoint              | Description                    |
|--------|-----------------------|--------------------------------|
| GET    | `/`                   | Serve web interface            |
| GET    | `/api/robot_arm/info` | Get robot arm information      |
| POST   | `/api/robot_arm/cmd`  | Send command to robot arm      |

### Command API

Send commands via POST to `/api/robot_arm/cmd`:

```json
{
  "command": "power_on"
}
```

Response:
```json
{
  "status": "success",
  "command": "power_on"
}
```

---

## 🧪 Testing

### Manual Testing

1. Start the robot arm node and web server
2. Open browser to `http://localhost:8080`
3. Click buttons to test commands
4. Check ROS logs for command reception

### Unit Tests

```bash
colcon test --packages-select robot_arm_web
```

### Integration Testing

```bash
# Terminal 1: Start robot arm node
ros2 run duco_robot_arm duco_robot_arm_node --ros-args -p ip:=192.168.1.10

# Terminal 2: Start web server
ros2 run robot_arm_web robot_arm_web_server

# Terminal 3: Monitor commands
ros2 topic echo /arm1/cmd
```

---

## 🔧 Development

### Adding New Commands

1. Update the available commands in `web_server_node.py`
2. Add button in `index.html`
3. Add JavaScript handler in `robot_arm_control.js`
4. Update the `duco_robot_arm_node.py` to handle new commands

### Customizing the Interface

* Edit `web/index.html` for layout changes
* Modify `web/css/style.css` for styling
* Update `web/js/robot_arm_control.js` for functionality

---

## 🛡️ Security Considerations

⚠️ **Important Security Notes**:

1. **Network Access**: Web server binds to `0.0.0.0` (all interfaces)
2. **Authentication**: No authentication implemented - use in trusted networks
3. **HTTPS**: Not configured - consider adding for production use
4. **Firewall**: Ensure port 8080 is properly configured

---

## 🐛 Troubleshooting

### Common Issues

**Web Server Won't Start**:
```bash
# Check if port is already in use
sudo netstat -tulpn | grep :8080

# Try different port
ros2 run robot_arm_web robot_arm_web_server --ros-args -p port:=8081
```

**Commands Not Working**:
```bash
# Check if robot arm node is running
ros2 node list | grep duco_robot_arm

# Check topic
ros2 topic echo /arm1/cmd
```

**Browser Can't Connect**:
```bash
# Check firewall
sudo ufw status

# Check if server is running
curl http://localhost:8080/api/robot_arm/info
```

---

## 📄 License

MIT License

---

## 👤 Maintainer

**Yi-Zhong Zhang** - [yizhongzhang1989@gmail.com](mailto:yizhongzhang1989@gmail.com)

---

## 🔗 Related Packages

* `duco_robot_arm` - Robot arm control node
* `robot_bringup` - Complete robot system launch
* `robot_web` - Motor and servo web interface

---

## 🚀 Future Enhancements

* 🔐 Add authentication and authorization
* 📱 Mobile-responsive improvements
* 🎥 Add robot arm status visualization
* 🔧 More advanced control options
* 📊 Command history and analytics
