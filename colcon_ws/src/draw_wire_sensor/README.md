# draw_wire_sensor

Renamed from lift_robot_cable_sensor. Publishes draw wire (cable) sensor registers over ROS2.

## Launch
```bash
ros2 launch draw_wire_sensor draw_wire_sensor.launch.py device_id:=51 read_interval:=0.1
```
Parameters:
- `device_id` (int): Modbus device id
- `read_interval` (double): seconds between reads
