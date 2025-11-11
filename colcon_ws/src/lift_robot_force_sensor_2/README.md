# lift_robot_force_sensor_2

Second single-channel force sensor node (device_id=53) for the lift robot. Publishes raw force to a distinct topic to allow running alongside the primary sensor.

## Function
- Reads one 16-bit holding register (FC03) at address 0x0000 from slave ID 53.
- Interprets unsigned 16-bit value as Newtons (raw). Calibration can be added later.
- Optional 50 FPS OpenCV visualization window (`ForceSensor2Live`).

## Topic
| Name | Type | Description |
|------|------|-------------|
| `/force_sensor_2` | `std_msgs/Float32` | Raw force value from second sensor |

## Parameters
| Name | Default | Description |
|------|---------|-------------|
| `device_id` | 53 | Modbus slave ID of second force sensor |
| `use_ack_patch` | True | Use ack patch with Modbus driver |
| `read_interval` | 0.02 | Read period (50 Hz) |
| `enable_visualization` | False | Enable OpenCV real-time plot |
| `calibration_scale` | 0.023614 | Calibration scale factor (from calibration) |
| `calibration_offset` | 0.0 | Calibration offset (0 after tare) |

## Calibration

The sensor publishes **calibrated force values** using the formula:
```
actual_force = sensor_reading × calibration_scale + calibration_offset
```

Default calibration (device_id=53):
- `scale = 0.023614`
- `offset = 0.0`

To recalibrate:
1. Run tare: `python3 scripts/force_sensor_tare.py --device_id 53`
2. Run calibration: `python3 scripts/force_sensor_calibrate.py 53`
3. Update launch file with new `calibration_scale` value

## Launch
```bash
ros2 launch lift_robot_force_sensor_2 lift_robot_force_sensor_launch.py
```

## Example Log Line
```
[INFO] [lift_robot_force_sensor_2]: [Sensor2 SEQ 42] FORCE reg: 0x00C8 value=200N
```

## Notes
- Make sure the Modbus bus can sustain combined 50Hz polling for all sensors; raise `read_interval` if saturation occurs.
- To visualize: `ros2 run lift_robot_force_sensor_2 force_sensor_node_2 --visualize` (or add parameter `enable_visualization:=true`).

## Future Extensions
- Calibration and scaling factors.
- Diagnostic status + stale data detection.
- Combine with primary sensor for differential or summed force analysis.
