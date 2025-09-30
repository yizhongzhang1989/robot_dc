# Cartesian Controller Node

This repository provides a set of tools, launch files, and utility nodes for using Cartesian controllers with a robot equipped with a force/torque (F/T) sensor. It includes support for joystick teleoperation, calibration, force control, compliance control, and motion control.

---

## 1. Aliases

For simplicity, convenient aliases are defined in `a@msraig-robot-02:~/.bashrc`:

```bash
alias myrun="ros2 launch duco_gcr5_910_moveit_config cartesian_controller.launch.py robot_ip:=192.168.1.10"
alias myswitch_to_force="ros2 control switch_controllers --deactivate cartesian_compliance_controller cartesian_motion_controller --activate cartesian_force_controller"
alias myswitch_to_compliance="ros2 control switch_controllers --deactivate cartesian_force_controller cartesian_motion_controller --activate cartesian_compliance_controller"
alias myswitch_to_motion="ros2 control switch_controllers --deactivate cartesian_force_controller cartesian_compliance_controller --activate cartesian_motion_controller"
alias mybuild="colcon build --packages-select duco_gcr5_910_moveit_config cartesian_controller_node && source install/setup.bash"
alias myjoy="ros2 run joy joy_node"
alias mycalib="python3 ft_calibrator.py --dir calib --out ft_calib_result.json"
```

> **Tip:** In the usage examples below, always prefer using these aliases for simplicity.

---

## 2. Build Instructions

- To build required packages:
  ```bash
  mybuild
  ```

- The `duco_gcr5_910_moveit_config` package (from branch `duco_ros2_driver`) contains modifications required for Cartesian controllers:
  - `config/cartesian_controller_manager.yaml`
  - `launch/cartesian_controller.launch.py`

These files configure and launch the Cartesian controllers.

---

## 3. Controllers Overview

- **`cartesian_motion_controller`**  
  Follows commanded Cartesian motions.

- **`cartesian_force_controller`**  
  Tracks external forces applied to the robot.

- **`cartesian_compliance_controller`**  
  Balances motion and force tracking. The stiffness parameters in  
  `config/cartesian_controller_manager.yaml` control the balance.

> The parameters in `cartesian_controller_manager.yaml` have been tuned for stable operation. Start with these values and adjust gradually if needed.

---

## 4. Utility Nodes

Located under `branch_cartesian_control/cartesian_controller_node/cartesian_controller_node`:

- **`zero_force_control_node.py`**  
  Publishes zero vectors to `/target_wrench`. Used for zero force control.

- **`joystick_calibration_node.py`**  
  Controls robot motion using a joystick during calibration.

- **`joystick_compliance_control_node.py`**  
  Controls motion with a joystick. Pressing the **A button** applies a downward 20N force.  
  The balance between motion and force is determined by stiffness parameters in  
  `duco_gcr5_910_moveit_config/config/cartesian_controller_manager.yaml`.

- **`joystick_force_control_node.py`**  
  Controls force application via joystick. (Rotation control may not yet be implemented.)

- **`calibration_data_recorder.py`**  
  Records poses and F/T sensor values for calibration.

- **`ft_calibrator.py`**  
  Processes calibration data and outputs a calibration file.  
  Example:
  ```bash
  python3 ft_calibrator.py --dir calib --out ft_calib_result.json
  ```

---

## 5. Usage Guide

### Calibration
1. Ensure the `calib` folder (user-specified) is empty before starting.
2. Run:
   ```bash
   myrun
   myjoy
   myswitch_to_motion
   ros2 launch cartesian_controller_node joystick_calibration.launch.py
   ros2 launch cartesian_controller_node calibration_data_recorder.launch.py
   ```
3. Use the joystick to rotate the robot randomly.  
   Every 5 seconds, the system records pose and F/T sensor data.  
   Collect ~10–20 points.
4. Generate calibration result:
   ```bash
   mycalib
   ```
   → Outputs `ft_calib_result.json` inside `calib/`.

---

### Joystick Teleoperation
```bash
myrun
myjoy
myswitch_to_{compliance|force}
ros2 launch cartesian_controller_node joystick_{compliance|force}_control.launch.py
```
- Control the robot with the joystick in **compliance** or **force** mode.

---

### Direct Manual Control

**(a) Zero Force Control**
```bash
myrun
myswitch_to_force
ros2 launch cartesian_controller_node zero_force_control.launch.py
```

**(b) Compliance Control**
```bash
myrun
myswitch_to_compliance
ros2 launch cartesian_controller_node zero_force_control.launch.py
```

- In zero force mode, the robot follows external forces.  
- In compliance mode, the robot follows external forces but returns to its original position once released (due to compliance stiffness).  

You can configure constrained axes via:
```yaml
config/cartesian_controller_manager.yaml   # see lines 87–94
```

---

## 6. Troubleshooting

- **Symptom:** Robot drifts in force/compliance control.  
  **Cause:** F/T sensor not calibrated.  
  **Solution:** Repeat the calibration procedure.


