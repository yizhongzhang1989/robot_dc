# robot_arm_teleop

`robot_arm_teleop` 是一个ROS2包，通过手柄（游戏手柄）实现对机械臂的遥控操作。它订阅来自 `joy` 包的输入，并将按钮和摇杆操作转换为机械臂命令。

---

## 📦 包结构

```
robot_arm_teleop/
├── robot_arm_teleop/
│   ├── __init__.py
│   └── robot_arm_teleop_node.py     # 主要的遥控节点
├── launch/
│   └── robot_arm_teleop_launch.py   # 启动文件
├── config/
│   └── joy_params.yaml              # 手柄配置参数
├── package.xml
├── setup.py
└── README.md
```

---

## 🎮 功能特性

### 🕹️ 手柄控制映射

#### 摇杆控制 (Servo模式)
- **左摇杆垂直** (axes[1]) → 控制关节1角度
- **左摇杆水平** (axes[0]) → 控制关节2角度  
- **右摇杆垂直** (axes[3]) → 控制关节3角度
- **右摇杆水平** (axes[2]) → 控制关节4角度
- **左扳机** (axes[4]) → 控制关节5角度
- **右扳机** (axes[5]) → 控制关节6角度

#### TCP笛卡尔坐标控制 (按钮模式)
- **方向键上/下** → X轴位置控制
- **方向键左/右** → Y轴位置控制  
- **LB/RB按钮** → Z轴位置控制
- **LT/RT按钮** → Rx旋转控制

#### 按钮功能
- **A按钮** → 机械臂使能/去使能切换
- **B按钮** → 紧急停止
- **X按钮** → 回零位置
- **Y按钮** → 切换控制模式（关节/TCP）
- **Start按钮** → 上电
- **Back按钮** → 断电

---

## ⚙️ 依赖项

- ROS2 (Humble推荐)
- sensor_msgs
- std_msgs
- joy包
- duco_robot_arm包

---

## 🚀 使用方法

### 1. 编译包

```bash
cd ~/colcon_ws
colcon build --packages-select robot_arm_teleop
source install/setup.bash
```

### 2. 连接手柄

确保手柄已连接并被系统识别：

```bash
ls /dev/input/js*
jstest /dev/input/js0
```

### 3. 启动遥控

使用launch文件启动完整系统：

```bash
ros2 launch robot_arm_teleop robot_arm_teleop_launch.py
```

或者分别启动：

```bash
# 终端1：启动手柄节点
ros2 run joy joy_node

# 终端2：启动机械臂遥控节点
ros2 run robot_arm_teleop robot_arm_teleop

# 终端3：启动机械臂节点（如果还没启动）
ros2 run duco_robot_arm duco_robot_arm_node
```

---

## 🎯 机械臂命令话题

遥控节点发布命令到：

- `/arm1/cmd` → `std_msgs/String`

### 发布的命令示例：

**关节控制模式：**
- `servoj [j1,j2,j3,j4,j5,j6] 1.0 1.0 False 200 65`

**TCP控制模式：**
- `servo_tcp [x,y,z,rx,ry,rz] 1.0 1.0 "" False 150 35`

**基本控制命令：**
- `power_on` / `power_off`
- `enable` / `disable`

---

## 🧪 测试

### 测试手柄输入

```bash
jstest /dev/input/js0
```

或使用图形界面：

```bash
jstest-gtk
```

### 查看手柄原始话题

**终端1** (启动手柄节点)：
```bash
ros2 run joy joy_node
```

**终端2** (查看话题)：
```bash
ros2 topic echo /joy
```

你应该能看到手柄操作的实时更新。

### 查看机械臂命令

```bash
ros2 topic echo /arm1/cmd
```

---

## 🛠️ 配置 (可选)

### 手柄参数配置

编辑 `config/joy_params.yaml` 来调整手柄设置：

```yaml
joy_node:
  ros__parameters:
    device_id: 0
    deadzone: 0.05
    autorepeat_rate: 20.0
```

### 控制参数调整

在节点代码中可以调整以下参数：
- 关节角度步长
- TCP位置步长  
- 速度和加速度限制
- 死区阈值

---

## ⚠️ 安全注意事项

1. **首次使用前**请确保机械臂周围无障碍物
2. **始终准备好紧急停止**（B按钮）
3. **小幅度测试**后再进行大范围动作
4. **确保机械臂已正确标定**和初始化

---

## 📄 许可证

MIT License

---

## 👤 维护者

robot_arm_teleop 
Email: yizhongzhang1989@example.com
