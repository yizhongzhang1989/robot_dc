# modbus_driver

`modbus_driver` is a ROS 2 package that provides a centralized Modbus RTU interface for communicating with Modbus slave devices (e.g., motors, sensors) over a serial bus. It exposes a ROS 2 service for read/write operations, allowing multiple nodes to share a single Modbus bus without contention.

---

## 📦 Package Structure

```

modbus\_driver/
├── modbus\_driver/
│   ├── modbus\_manager\_node.py         # Main Modbus manager node
│   ├── modbus\_client\_tester.py        # Optional standalone tester script
├── launch/
│   └── modbus\_manager\_launch.py       # Launch file to start the manager node
├── test/
│   └── test\_modbus\_manager.py         # Unit tests
modbus\_driver\_interfaces/
└── srv/
└── ModbusRequest.srv              # Custom service for Modbus requests

````

---

## ⚙️ Features

- ✅ Read Holding Registers (Function Code 3)
- ✅ Write Single Register (Function Code 6)
- ✅ Write Multiple Registers (Function Code 16)
- ✅ Thread-safe access to serial bus
- ✅ Configurable serial port and baud rate via ROS parameters

---

## 🛠️ Dependencies

- ROS 2 (tested with **Humble**)
- [pymodbus](https://github.com/riptideio/pymodbus)

Install Python dependency:

```bash
pip install pymodbus
````

---

## 🚀 Usage

### 1. Build the Workspace

From the root of your workspace:

```bash
colcon build
source install/setup.bash
```

### 2. Launch the Modbus Manager

```bash
ros2 launch modbus_driver modbus_manager_launch.py
```

By default, this uses:

* Port: `/dev/ttyUSB0`
* Baudrate: `38400`

These can be overridden by passing arguments to the launch file or defining parameters in a YAML config.

### 3. Send a Modbus Request

```bash
ros2 service call /modbus_request modbus_driver_interfaces/srv/ModbusRequest \
"{function_code: 3, slave_id: 1, address: 0, count: 2, values: []}"
```

#### 🧾 Example: Write Single Register

```bash
ros2 service call /modbus_request modbus_driver_interfaces/srv/ModbusRequest \
"{function_code: 6, slave_id: 1, address: 0, count: 1, values: [123]}"
```

---

## 🧪 Testing

### Manual Testing

```bash
ros2 run modbus_driver modbus_client_tester
```

This script can be used to send raw Modbus requests for quick validation.

### Unit Tests

```bash
colcon test --packages-select modbus_driver
```

Mocks the Modbus client, so hardware is not required.

---

## 🧾 Service Definition

**`ModbusRequest.srv`**

```plaintext
uint8 function_code
uint8 slave_id
uint16 address
uint16 count
uint16[] values
---
bool success
uint16[] response
```

---

## 📄 License

MIT License

---

## 👤 Maintainer

[jetson@todo.todo](mailto:jetson@todo.todo)

