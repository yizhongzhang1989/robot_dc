import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from modbus_driver_interfaces.msg import MotorSimulationStatus
from threading import Thread, Lock

class WebROSClient:
    def __init__(self, motor_list):
        self.motor_list = motor_list
        self.node = None
        self.publishers = {}
        self.status_map = {}
        self.lock = Lock()
        self._start_ros_node()

    def _start_ros_node(self):
        def ros_spin():
            rclpy.init()
            self.node = Node("web_ros_client")

            for motor in self.motor_list:
                cmd_topic = f"/{motor}/cmd"
                status_topic = f"/{motor}/sim_status"

                # Publisher for sending commands
                self.publishers[motor] = self.node.create_publisher(String, cmd_topic, 10)

                # Subscriber for receiving sim_status
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

    def get_motor_status(self, motor_id):
        with self.lock:
            return self.status_map.get(motor_id, None)
