# robot\_teleop

`robot_teleop` is a ROS 2 package that enables teleoperation of your robot using a joystick (gamepad). It subscribes to joystick input via the `joy` package and translates buttons and axes into motor commands, publishing them to specific motor topics.

---

## 📦 Package Structure

```
robot_teleop/
├── robot_teleop/
│   └── joystick_teleop_node.py     # Main teleop node that maps joystick input to motor commands
├── launch/
│   └── joystick_teleop_launch.py   # Launch file to start teleop + joy node
├── config/
│   └── joy_params.yaml             # Joystick configuration (optional, for advanced settings)
```

---

## 🎮 Features

* ✅ Button 0: Jog Motor 1 left while held
* ✅ Button 1: Jog Motor 2 left while held
* ✅ Axis 1: Control Motor 1 velocity (scaled and signed)
* ✅ Axis 3: Control Motor 2 velocity (scaled and signed)
* ✅ Deadzone filtering to avoid noisy axis changes
* ✅ Separate motor command topics for modularity

---

## ⚙️ Dependencies

* ROS 2 (**Humble**)
* [`joy`](https://github.com/ros2/joystick_drivers/tree/humble/joy) ROS package

Install required packages:

```bash
sudo apt install ros-humble-joy
sudo apt install joystick jstest-gtk evtest
```

---

## 🚀 Usage

### 1. Build the Workspace

```bash
colcon build
source install/setup.bash
```

### 2. Launch the Teleop System

```bash
ros2 launch robot_teleop joystick_teleop_launch.py
```

This will:

* Start the `joy_node` (reads joystick input)
* Start the `joystick_teleop_node` (converts input into motor commands)

---

## 🎯 Motor Command Topics

The teleop node publishes motor commands to:

* `/motor1/motor_cmd` → `std_msgs/String`
* `/motor2/motor_cmd` → `std_msgs/String`

### Example Commands Published:

* 'set_vel -50'
* 'move_vel'
* 'stop'
* 'jog_left'

These topics should be subscribed by motor controller nodes such as `leadshine_motor`.

---

## 🧪 Testing

### Test Joystick

```bash
jstest /dev/input/js0
```

Or with GUI:

```bash
jstest-gtk
```

### Inspect Raw Joy Topic

> Requires two terminals

**Terminal 1** (start joy node):

```bash
ros2 run joy joy_node
```

**Terminal 2** (echo topic):

```bash
ros2 topic echo /joy
```

You should see live updates of axes and buttons.

---

## 🛠️ Configuration (Optional)

You can use a config file to adjust joystick parameters like deadzone:

**Example: `config/joy_params.yaml`**

```yaml
joy_node:
  ros__parameters:
    deadzone: 0.05
    autorepeat_rate: 20.0
```

**Launch with config:**

```bash
ros2 launch robot_teleop joystick_teleop_launch.py use_config:=true
```

---

## 📄 License

MIT License

---

## 👤 Maintainer

[jetson@todo.todo](mailto:jetson@todo.todo)

