#!/usr/bin/env python3
"""
Lift Robot Server Positioning Script

Task (Force-threshold mode):
    1. Initialization: Move platform DOWN for fixed duration, stop, wait for height stabilization (capture initial_height).
    2. Raise platform continuously (command "up") at 50 Hz control loop; each loop compute filtered force:
         - Take last 4 raw force samples.
         - Discard max and min.
         - Average the remaining 2 -> filtered_force.
       Stop when filtered_force >= force_threshold (default 750 N) or timeout.
       (Set --force-threshold -1 to disable force mode and instead goto (initial_height + offset_mm).)
    3. Output JSON summary.

Produces JSON summary on stdout:
{
    "initial_height": <float>,
    "target_height": <float>,
    "reached_height": <float>,
    "status": "success"|"error",
    "error": <str or null>
}

Run after sourcing ROS2 workspace:
  source install/setup.bash
    python3 scripts/lift_robot_server_positioning.py --offset-mm 15

Dry run (no motion, simulates readings):
    python3 scripts/lift_robot_server_positioning.py --dry-run

Assumptions:
    - Platform 'down' moves toward mechanical base safely within platform_down_duration.
    - Platform accepts JSON String messages on topic /lift_robot_platform/command:
                {"command": "down"}
                {"command": "stop"}
                {"command": "goto_height", "target_height": 830.0}
    - Height sensor topic: /draw_wire_sensor/data JSON with 'height'.

Edge Cases Handled:
  - Missing sensor data -> retries & timeouts.
    - Height not stabilizing -> uses last known average.
    - Dry-run mode returns synthetic values.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json, time, sys, argparse, math

class PositioningNode(Node):
    def __init__(self, args):
        super().__init__('lift_robot_server_positioning')
        self.args = args
        # Publisher for platform commands
        self.platform_cmd_pub = self.create_publisher(String, '/lift_robot_platform/command', 10)
        # Height sensor
        self.height = None
        self.height_history = []
        self.height_sub = self.create_subscription(String, '/draw_wire_sensor/data', self.height_cb, 10)
        # Force sensor
        self.force = None
        self.force_history = []  # raw force samples
        self.force_sub = self.create_subscription(Float32, '/force_sensor', self.force_cb, 10)
        # Results
        self.initial_height = None
        self.target_height = None  # Only used in height-target mode
        self.reached_height = None
        self.final_force = None
        self.final_force_filtered = None

    def height_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            h = data.get('height')
            if h is not None:
                self.height = float(h)
                self.height_history.append(self.height)
                if len(self.height_history) > 50:  # keep last 50 for stability check
                    self.height_history.pop(0)
        except Exception as e:
            self.get_logger().warn(f"Height parse error: {e}")

    def publish_cmd(self, publisher, payload: dict, label: str):
        if self.args.dry_run:
            self.get_logger().info(f"[DRY-RUN] Would publish to {label}: {payload}")
            return
        msg = String()
        msg.data = json.dumps(payload)
        publisher.publish(msg)
        self.get_logger().info(f"Published {label} command: {payload}")

    def force_cb(self, msg: Float32):
        try:
            v = float(msg.data)
            self.force = v
            self.force_history.append(v)
            # Keep only last 20 samples (enough for multiple decisions)
            if len(self.force_history) > 20:
                self.force_history.pop(0)
        except Exception as e:
            self.get_logger().warn(f"Force parse error: {e}")

    def filtered_force(self):
        """Return filtered force using last 4 samples: discard max & min, average middle two.
        If fewer than 4 samples, return None."""
        if len(self.force_history) < 4:
            return None
        last4 = self.force_history[-4:]
        sorted_vals = sorted(last4)
        mid2 = sorted_vals[1:3]
        return sum(mid2) / 2.0

    def wait_for_height(self, timeout):
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.height is not None:
                return True
        return False

    def stable_height(self, window=10, tolerance=0.3):
        if len(self.height_history) < window:
            return None
        sample = self.height_history[-window:]
        avg = sum(sample)/len(sample)
        max_dev = max(abs(h-avg) for h in sample)
        if max_dev <= tolerance:
            return avg
        return None

    def perform_initialization(self):
        self.get_logger().info("Phase 1: Initialization - moving platform down to base (pushrod removed)")
        self.publish_cmd(self.platform_cmd_pub, {"command": "down"}, 'platform')
        plat_dur = self.args.platform_down_duration
        t0 = time.time(); plat_stopped = False
        while rclpy.ok():
            elapsed = time.time() - t0
            rclpy.spin_once(self, timeout_sec=0.05)
            if not plat_stopped and elapsed >= plat_dur:
                self.publish_cmd(self.platform_cmd_pub, {"command": "stop"}, 'platform')
                plat_stopped = True
                self.get_logger().info(f"Platform down duration {plat_dur:.1f}s elapsed -> STOP")
            if plat_stopped:
                break
        if not plat_stopped:
            self.publish_cmd(self.platform_cmd_pub, {"command": "stop"}, 'platform')
        if not self.wait_for_height(timeout=5):
            raise RuntimeError("No height data received after initialization")
        # Stability check
        stab_start = time.time(); stable = None
        while rclpy.ok() and time.time() - stab_start < self.args.stability_wait:
            rclpy.spin_once(self, timeout_sec=0.05)
            c = self.stable_height()
            if c is not None:
                stable = c; break
        if stable is None:
            stable = self.height
            self.get_logger().warn("Using last height (not stable) as initial height")
        self.initial_height = stable
        self.get_logger().info(f"Initial stable/base height: {self.initial_height:.2f} mm")

    def raise_to_target(self):
        # Decide motion strategy: force-stop by default per requirement.
        if self.args.force_threshold is not None and self.args.force_threshold >= 0:
            thresh = self.args.force_threshold
            self.get_logger().info(f"Phase 2: Raising platform until filtered force >= {thresh:.1f} N (4-sample middle-average filter)")
            # Start continuous up motion
            self.publish_cmd(self.platform_cmd_pub, {"command": "up"}, 'platform')
            start = time.time()
            exceeded = False
            loop_period = 1.0 / 50.0  # 50 Hz control loop
            while rclpy.ok() and time.time() - start < self.args.adjust_timeout:
                rclpy.spin_once(self, timeout_sec=loop_period)
                # Dry-run: synthesize increasing force for demonstration
                if self.args.dry_run and (self.force is None or len(self.force_history) < 4):
                    base = 500.0
                    inc = (time.time() - start) * 80.0  # ~80N per second
                    synthetic = base + inc
                    self.force = synthetic
                    self.force_history.append(synthetic)
                    if len(self.force_history) > 20:
                        self.force_history.pop(0)
                f_filt = self.filtered_force()
                if f_filt is not None:
                    if f_filt >= thresh:
                        exceeded = True
                        self.get_logger().info(f"Force threshold reached: filtered {f_filt:.2f} >= {thresh:.2f}")
                        break
            self.publish_cmd(self.platform_cmd_pub, {"command": "stop"}, 'platform')
            if not exceeded:
                self.get_logger().warn("Force threshold not reached before timeout")
            self.reached_height = self.height
            self.final_force = self.force
            self.final_force_filtered = self.filtered_force()
        else:
            # Fallback to height target mode
            self.target_height = self.initial_height + self.args.offset_mm
            self.get_logger().info(f"Phase 2: Raising platform to target_height {self.target_height:.2f} mm (height mode fallback)")
            self.publish_cmd(self.platform_cmd_pub, {"command": "goto_height", "target_height": self.target_height}, 'platform')
            start = time.time(); reached=False
            while rclpy.ok() and time.time() - start < self.args.adjust_timeout:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.height is not None and abs(self.height - self.target_height) <= self.args.height_tolerance:
                    reached=True; break
            if not reached:
                self.get_logger().warn("Raise timeout; using last measured height")
            self.publish_cmd(self.platform_cmd_pub, {"command": "stop"}, 'platform')
            self.reached_height = self.height if self.height is not None else self.target_height
            self.final_force = self.force
            self.final_force_filtered = self.filtered_force()
            self.get_logger().info(f"Reached height: {self.reached_height:.2f} mm (target {self.target_height:.2f} mm)")

    # detection_phase removed in simplified version

    def summary(self, error=None):
        status = 'success' if error is None else 'error'
        return {
            'initial_height': self.initial_height,
            'target_height': self.target_height,
            'reached_height': self.reached_height,
            'final_force': self.final_force,
            'final_force_filtered': self.final_force_filtered,
            'force_threshold': self.args.force_threshold,
            'status': status,
            'error': error
        }

def parse_args():
    p = argparse.ArgumentParser(description='Lift Robot Server Positioning')
    p.add_argument('--offset-mm', type=float, default=15.0, help='Offset in mm added to initial base height for final target')
    p.add_argument('--platform-down-duration', type=float, default=15.0, help='Seconds to drive platform down to mechanical base')
    p.add_argument('--stability-wait', type=float, default=3.0, help='Seconds to attempt height stability detection')
    p.add_argument('--adjust-timeout', type=float, default=8.0, help='Timeout for raising (force or height mode)')
    p.add_argument('--height-tolerance', type=float, default=0.5, help='Tolerance (mm) for considering height reached')
    p.add_argument('--dry-run', action='store_true', help='Do not send motion commands; simulate height readings only')
    p.add_argument('--force-threshold', type=float, default=750.0, help='Force threshold (N) using 4-sample middle-average filter; platform stops when reached. Set to -1 to disable force mode.')
    return p.parse_args()

def main():
    args = parse_args()
    rclpy.init()
    node = PositioningNode(args)
    try:
        node.perform_initialization()
        node.raise_to_target()
        result = node.summary()
        print(json.dumps(result, indent=2))
    except Exception as e:
        result = {
            'initial_height': node.initial_height,
            'status': 'error',
            'error': str(e)
        }
        print(json.dumps(result, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
