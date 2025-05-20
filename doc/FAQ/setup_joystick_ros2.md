# Setting Up and Testing a Joystick with ROS 2

This guide walks through installing, configuring, and testing a joystick (game controller) for robot control using ROS 2 and the `joy` package.

---

## 1. Install Required Linux Packages

Install system utilities for detecting and testing joystick devices:

```bash
sudo apt install joystick jstest-gtk evtest
````

---

## 2. Connect and Verify Joystick Input (Linux)

### Check device events:

```bash
sudo evtest
```

* Choose the correct event number corresponding to your joystick.
* Press buttons or move axes — you should see input activity in the terminal.

If you get permission errors or no input is detected:

```bash
# Replace event0 with the actual device from evtest
sudo chmod a+rw /dev/input/event0
```

**Note:** This step is usually not necessary when using `/dev/input/jsX`, which the ROS `joy` node uses.

### Alternatively, test with `jstest`:

```bash
jstest /dev/input/js0
```

---

## 3. Special Notes for Logitech F310 Controller

The **Logitech F310 gamepad** includes a switch on the back labeled **"X"/"D"** to toggle between:

* **XInput mode ("X")** – behaves like an Xbox controller (may not expose `/dev/input/js0`)
* **DirectInput mode ("D")** – behaves like a traditional joystick and **is recommended for ROS 2**

To use the F310 with ROS 2:

1. **Flip the switch to the “D” position**.
2. **Reconnect the controller** (unplug and plug back in).
3. Confirm the device exists:

```bash
ls /dev/input/js*
```

You should now see `/dev/input/js0`.

4. Test input:

```bash
jstest /dev/input/js0
```

If `/dev/input/js0` is still missing, ensure the `joydev` module is loaded:

```bash
lsmod | grep joydev
```

---

## 4. Verify ROS 2 Can Detect the Joystick

```bash
ros2 run joy joy_enumerate_devices
```

This will list available joystick devices (e.g., `/dev/input/js0`).

---

## 5. Run the Joy Node and Echo Messages

In **Terminal 1**, start the joystick node:

```bash
ros2 run joy joy_node
```

In **Terminal 2**, check if `/joy` topic is publishing messages:

```bash
ros2 topic echo /joy
```

You should see a stream of joystick messages as you interact with the controller.

---

## 6. Launching with a Launch File

Use the existing `joystick.launch.py` in your repo’s launch directory (or adapt it if needed):

```python
# File: colcon_ws/src/<your_package>/launch/joystick.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_params = os.path.join(
        get_package_share_directory('<your_package>'),
        'config',
        'joystick.yaml'
    )

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joy_params]
        )
    ])
```

Replace `<your_package>` with the name of your package (e.g., `leadshine_motor` if that's where config is located).

Launch it using:

```bash
ros2 launch <your_package> joystick.launch.py
```

---

## 7. Joystick Configuration File (`joystick.yaml`)

Make sure the config file exists under your `config/` directory:

```yaml
# File: config/joystick.yaml

joy_node:
  ros__parameters:
    device_id: 0
    deadzone: 0.05
    autorepeat_rate: 20.0
```

You can adjust these parameters depending on your joystick behavior.

---

## 8. Additional Resources

* ROS 2 `joy` package documentation:
  [https://index.ros.org/p/joy/](https://index.ros.org/p/joy/)

---

## 9. Troubleshooting

* If `/joy` topic isn't publishing, confirm:

  * Joystick is detected (`ls /dev/input/js*`)
  * `joy_node` is running
  * `jstest` shows input activity
* For Logitech F310, make sure it's in **“D” mode**, not “X” mode.
* Try different `device_id` values if you have multiple input devices.
