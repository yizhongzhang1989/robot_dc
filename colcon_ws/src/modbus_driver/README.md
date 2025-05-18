# modbus_driver

`modbus_driver` is a ROS 2 package that provides a centralized Modbus RTU communication interface over a serial connection. It exposes a service (`/modbus_request`) for other ROS nodes to interact with Modbus slave devices like motors and sensors without directly accessing the serial port.

---

## 📦 Package Structure

- `modbus_manager_node.py`: Main node that handles service requests and interacts with serial devices.
- `modbus_rtu_interface.py`: Wrapper around `pymodbus` for Modbus RTU communication.
- `ModbusRequest.srv`: Custom ROS 2 service definition supporting read/write operations.
- `modbus_manager_launch.py`: Launch file to start the node.

---

## ⚙️ Features

- Read Holding Registers (FC 3)
- Write Single Register (FC 6)
- Write Multiple Registers (FC 16)
- Centralized resource access (prevents bus contention)
- Configurable serial port and baudrate

---

## 🚀 Usage

### Launch the Manager

```bash
ros2 launch modbus_driver modbus_manager_launch.py
````

### Call Service

```bash
ros2 service call /modbus_request modbus_driver/srv/ModbusRequest \
"{function_code: 3, slave_id: 1, address: 0, values: [0, 0]}"
```

---

## 🧪 Testing

### Manual Testing

```bash
ros2 run modbus_driver modbus_client_tester
```

### Unit Testing

```bash
colcon test --packages-select modbus_driver
```

Unit tests mock the Modbus interface, so no hardware is needed.

---

## 🛠️ Dependencies

* ROS 2 (tested with Humble/Foxy)
* Python `pymodbus`

Install with:

```bash
pip install pymodbus
```

---

## 📄 License

MIT

---

## 👤 Maintainer

[jetson@todo.todo](mailto:jetson@todo.todo)

```

