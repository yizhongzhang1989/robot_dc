import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from modbus_driver_interfaces.msg import MotorSimulationStatus
from threading import Thread, Lock

class WebROSClient:
    def __init__(self, motor_list):
        self.motor_list = motor_list
        self.node = None
        self.publishers = {}
        self.status_map = {}
        self.lock = Lock()
        self.obs_ctrl_publishers = {}  # Ensure this is an instance attribute
        self._start_ros_node()

    def _start_ros_node(self):
        def ros_spin():
            rclpy.init()
            self.node = Node("web_ros_client")

            # Observation control Publisher
            for motor_id in [1, 2]:
                ctrl_topic = f"/motor{motor_id}/read_ctrl"
                self.obs_ctrl_publishers[f"motor{motor_id}"] = self.node.create_publisher(String, ctrl_topic, 10)
            for servo_id in [17, 18]:
                ctrl_topic = f"/servo{servo_id}/read_ctrl"
                self.obs_ctrl_publishers[f"servo{servo_id}"] = self.node.create_publisher(String, ctrl_topic, 10)

            # Subscribe to motor position published by detector node
            for motor_id in [1, 2]:
                topic = f"/motor{motor_id}/position"
                self.node.create_subscription(
                    Int32, topic,
                    lambda msg, m=motor_id: self._update_motor_position(f"motor{m}", msg),
                    10
                )

            # Subscribe to servo position and torque published by detector node
            for servo_id in [17, 18]:
                pos_topic = f"/servo{servo_id}/position"
                pwm_topic = f"/servo{servo_id}/pwm"
                self.node.create_subscription(
                    Int32, pos_topic,
                    lambda msg, s=servo_id: self._update_servo_position(f"servo{s}", msg),
                    10
                )
                self.node.create_subscription(
                    Float32, pwm_topic,
                    lambda msg, s=servo_id: self._update_servo_torque(f"servo{s}", msg),
                    10
                )

            # Keep original functionality
            for motor in self.motor_list:
                cmd_topic = f"/{motor}/cmd"
                status_topic = f"/{motor}/sim_status"
                self.publishers[motor] = self.node.create_publisher(String, cmd_topic, 10)
                self.node.create_subscription(
                    MotorSimulationStatus,
                    status_topic,
                    lambda msg, m=motor: self._update_status(m, msg),
                    10
                )

            rclpy.spin(self.node)

        Thread(target=ros_spin, daemon=True).start()

    def send_command(self, motor_id, command, value=None):
        if motor_id not in self.publishers:
            return {"error": f"Unknown motor: {motor_id}"}
        cmd_str = f"{command} {value}" if value is not None else command
        msg = String()
        msg.data = cmd_str
        self.publishers[motor_id].publish(msg)
        return {"motor": motor_id, "command": cmd_str}

    def control_observation(self, target, action):
        # target: 'motor1', 'motor2', 'servo17', 'servo18'
        # action: 'start' or 'stop'
        pub = self.obs_ctrl_publishers.get(target)
        if pub is None:
            return {"error": f"Unknown target: {target}"}
        msg = String()
        msg.data = action
        pub.publish(msg)
        return {"target": target, "action": action, "result": "sent"}

    def _update_status(self, motor_id, msg: MotorSimulationStatus):
        with self.lock:
            self.status_map[motor_id] = {
                "motor_id": msg.motor_id,
                "current_position": msg.current_position,
                "target_position": msg.target_position,
                "velocity": msg.velocity,
                "motion_mode": msg.motion_mode,
                "moving": msg.moving,
                "last_command": msg.last_command,
            }

    def _update_motor_position(self, motor_key, msg):
        with self.lock:
            if motor_key not in self.status_map:
                self.status_map[motor_key] = {}
            self.status_map[motor_key]["position"] = msg.data

    def _update_servo_position(self, servo_key, msg):
        with self.lock:
            if servo_key not in self.status_map:
                self.status_map[servo_key] = {}
            self.status_map[servo_key]["position"] = msg.data

    def _update_servo_torque(self, servo_key, msg):
        with self.lock:
            if servo_key not in self.status_map:
                self.status_map[servo_key] = {}
            self.status_map[servo_key]["torque"] = msg.data

    def get_motor_status(self, motor_id):
        with self.lock:
            return self.status_map.get(motor_id, None)

    def get_all_status(self):
        with self.lock:
            return self.status_map.copy()
