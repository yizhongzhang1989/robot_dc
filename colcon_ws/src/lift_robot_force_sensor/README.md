# lift_robot_force_sensor

Single-channel force sensor Modbus reader for the lift robot.

## Function
- Sends Modbus FC03 (Read Holding Registers) to slave ID 52, address 0x0000, count 2.
- Combines two 16-bit registers into one 32-bit value and logs.

## Topics
(Currently no topics published; data only logged.)

## Parameters
| Name | Default | Description |
|------|---------|-------------|
| device_id | 52 | Modbus slave ID of force sensor |
| use_ack_patch | True | Integrate with ack patch mechanism |
| read_interval | 1.0 | Seconds between reads |

## Launch
```bash
ros2 launch lift_robot_force_sensor lift_robot_force_sensor_launch.py
```

## Example Log Line
```
[INFO] [lift_robot_force_sensor]: [SEQ 5] Force sensor read: reg0=0x1234 reg1=0xABCD raw32=305419896 (dec) ts=1734548123.123
```

## Future Extensions
- Publish a ROS message with force value.
- Apply scaling / calibration factor.
- Add diagnostic status topic.
