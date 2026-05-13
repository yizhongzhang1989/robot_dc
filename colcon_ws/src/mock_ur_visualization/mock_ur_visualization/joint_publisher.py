#!/usr/bin/env python3
"""
joint_publisher — polls a UR robot's primary URScript port and publishes
sensor_msgs/JointState at a configurable rate.

This is the "no ur_robot_driver / no ros2_control" path to RViz visualization.
Use it whenever you want to *see* the robot move in RViz but you control the
arm via direct URScript (e.g. ur_robot_arm.UR15Robot) rather than ROS
controllers.

The message is published on the *relative* topic ``joint_states`` so the
launch file can push the whole node under any namespace it likes
(``visualize.launch.py`` pushes it under ``/mock`` by default).

Parameters (declare on launch or via --ros-args -p name:=value):
  robot_ip   (str)   IP of the robot          [default: 192.168.1.16]
  port       (int)   URScript primary port    [default: 30002]
  rate_hz    (float) publish rate             [default: 30.0]
  joint_prefix (str) prefix prepended to each joint name [default: ""]
  frame_id   (str)   header.frame_id          [default: "base_link"]
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Re-use the proven URScript client from this repo.
from ur_robot_arm.ur15 import UR15Robot


# URScript reports joints in this order; the standard UR URDF uses these names.
UR_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class JointPublisher(Node):
    def __init__(self) -> None:
        super().__init__("mock_ur_joint_publisher")

        # ----- parameters -----
        self.declare_parameter("robot_ip", "192.168.1.16")
        self.declare_parameter("port", 30002)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("joint_prefix", "")
        self.declare_parameter("frame_id", "base_link")

        self.robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.rate_hz = float(self.get_parameter("rate_hz").get_parameter_value().double_value)
        prefix = self.get_parameter("joint_prefix").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.joint_names = [prefix + n for n in UR_JOINT_NAMES]

        # ----- publisher -----
        self.pub = self.create_publisher(JointState, "joint_states", 10)

        # ----- URScript socket (lazy reconnect on failure) -----
        self._robot: UR15Robot | None = None
        self._robot_lock = threading.Lock()
        self._reconnect_backoff = 0.5  # seconds

        # ----- timer -----
        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"polling {self.robot_ip}:{self.port} at {self.rate_hz:.1f} Hz → "
            f"<ns>/joint_states (frame_id={self.frame_id}, prefix='{prefix}')"
        )

    # ------------------------------------------------------------------
    def _ensure_connected(self) -> bool:
        """Open the URScript socket if needed. Returns True on success."""
        with self._robot_lock:
            if self._robot is not None:
                return True
            try:
                r = UR15Robot(self.robot_ip, self.port)
                r.open()
                self._robot = r
                self._reconnect_backoff = 0.5
                return True
            except Exception as exc:
                self.get_logger().warning(
                    f"connect to {self.robot_ip}:{self.port} failed: {exc}; "
                    f"retrying in {self._reconnect_backoff:.1f}s"
                )
                time.sleep(self._reconnect_backoff)
                self._reconnect_backoff = min(self._reconnect_backoff * 2.0, 5.0)
                return False

    def _drop_connection(self) -> None:
        with self._robot_lock:
            r = self._robot
            self._robot = None
        if r is not None:
            try:
                r.close()
            except Exception:
                pass

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if not self._ensure_connected():
            return
        try:
            assert self._robot is not None
            q = self._robot.get_actual_joint_positions()
        except Exception as exc:
            self.get_logger().warning(f"read error: {exc}; will reconnect")
            self._drop_connection()
            return

        if q is None or len(q) != 6 or any(math.isnan(v) for v in q):
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.name = self.joint_names
        msg.position = [float(v) for v in q]
        self.pub.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self) -> bool:
        self._drop_connection()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
